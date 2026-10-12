/* Program: this is the source code for the IoT project of Chapter 08 of "Hands-On Low-Power Embedded Systems" book.
 *          Here, the focus is to optimize the IoT project's current / power / energy consumption by using ISR techniques
 *          in LoRaWAN's module AT commands response handling.
 *
 *          The device periodically reads a BMP180 (temperature and barometric pressure) sensor and an HC-SR04
 *          ultrasonic sensor (used here to measure water level / distance to a water surface), evaluates the
 *          readings against hazard thresholds, sends the results over LoRaWAN, and then powers down the sensors
 *          and enters deep sleep for one hour to save energy before repeating the whole cycle again on wake-up.
 *
 * Author: Pedro Bertoleti
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

/* Deep sleep header file */
#include "esp_sleep.h"

/* Power management header file */
#include "esp_pm.h"

/* Header files for needed libraries */
#include "bmp180.h"
#include "ultrasonic.h"

/* Header file for LoRaWAN communication */
#include "lorawan.h"

/* GPIOs used for I²C communication with the BMP180 sensor */
#define SDA_PIN 18
#define SCL_PIN 19

/* GPIOs used for communication with the ultrasonic (HC-SR04) sensor */
#define TRIG_PIN 5
#define ECHO_PIN 21

/* GPIO that drives a MOSFET switching the peripherals' power rail on/off, used to fully
 * cut power to the peripherals between measurement cycles and save energy during deep sleep */
#define GPIO_TURN_PERIPHERALS_ON_OFF 17

/* GPIO used to control the onboard LED */
#define GPIO_ONBOARD_LED 2

/* Combined bit-mask (used by the GPIO driver config) covering both the peripherals
 * power-control GPIO and the onboard LED GPIO */
#define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_TURN_PERIPHERALS_ON_OFF) | (1ULL << GPIO_ONBOARD_LED))

/* Thresholds for hazards */
/* Temperature (Fahrenheit): below LOW_TEMP_THRESHOLD or above HIGH_TEMP_THRESHOLD is considered a hazard */
#define LOW_TEMP_THRESHOLD 23.0
#define HIGH_TEMP_THRESHOLD 95.0

/* Barometric pressure (Hectopascal): below LOW_PRESSURE_THRESHOLD or above HIGH_PRESSURE_THRESHOLD is considered a hazard */
#define LOW_PRESSURE_THRESHOLD 940.0
#define HIGH_PRESSURE_THRESHOLD 970.0

/* Water level, expressed as the distance (in meters) measured by the ultrasonic sensor between itself
 * and the water surface. The larger the distance, the lower the water level, and vice-versa:
 * - distance greater than LOW_WATER_LEVEL_THRESHOLD  -> water level is too low  (hazard)
 * - distance lower than HIGH_WATER_LEVEL_THRESHOLD    -> water level is too high (flood hazard)
 */
#define LOW_WATER_LEVEL_THRESHOLD 12.0
#define HIGH_WATER_LEVEL_THRESHOLD 10.0

/*  Definitions: deep sleep time duration (in seconds), and the factor used to convert
 *  it to microseconds, since esp_sleep_enable_timer_wakeup() expects a duration in microseconds */
#define DEEP_SLEEP_DURATION       3600ULL   // seconds
#define DEEP_SLEEP_US_TOS_FACTOR  1000000ULL

/* Debug tags.
 * IMPORTANT: every ESP_LOGx() call in this project is of purely diagnostic purpose. ESP-IDF compiles
 * each of them conditioned to the compile-time log level (CONFIG_LOG_DEFAULT_LEVEL, which also sets
 * CONFIG_LOG_MAXIMUM_LEVEL / LOG_LOCAL_LEVEL): calls above that level are removed from the binary,
 * including their string formatting and arguments. Development builds use the default "sdkconfig"
 * (INFO level). Production builds use the "sdkconfig.prod.defaults" overlay, which sets the log level
 * to NONE (ESP_LOG_NONE), so no log call costs UART activity or CPU cycles in the field, without
 * having to manually remove any log line from the source code (see sdkconfig.prod.defaults for how to build it) */
static const char *TAG_LOGS_BMP180 = "BMP180";
static const char *TAG_LOGS_HCSR04 = "HC-SR04";

/* Function prototypes */
void check_hazards(char *pt_low_temp_hazard, char *pt_high_temp_hazard,
                   char *pt_low_pressure_hazard, char *pt_high_pressure_hazard,
                   char *pt_low_water_level_hazard, char *pt_high_water_level_hazard,
                   float temp, uint32_t pressure, float distance);
void isolate_sensor_signal_pins(void);

/* Function: check whether the current temperature, barometric pressure and water level
 *           readings indicate any hazard condition, by comparing them against the
 *           thresholds defined above
 * Parameters: - pointers to the hazard flags (low/high temperature, low/high barometric
 *               pressure and low/high water level), which are set to 0x01 when the
 *               corresponding hazard is detected, or 0x00 otherwise
 *             - current readings of temperature, barometric pressure and water level
 *               (distance from the ultrasonic sensor to the water surface)
 * Return: none (results are returned through the pointer parameters)
 */
void check_hazards(char *pt_low_temp_hazard, char *pt_high_temp_hazard,
                   char *pt_low_pressure_hazard, char *pt_high_pressure_hazard,
                   char *pt_low_water_level_hazard, char *pt_high_water_level_hazard,
                   float temp, uint32_t pressure, float distance)
{
    /* Check for temperature hazard */
    *pt_low_temp_hazard = 0x00;
    *pt_high_temp_hazard = 0x00;

    if (temp < LOW_TEMP_THRESHOLD)
        *pt_low_temp_hazard = 0x01;
    else if (temp > HIGH_TEMP_THRESHOLD)
        *pt_high_temp_hazard = 0x01;

    /* Check for barometric pressure hazard */
    *pt_low_pressure_hazard = 0x00;
    *pt_high_pressure_hazard = 0x00;

    if (pressure < LOW_PRESSURE_THRESHOLD)
        *pt_low_pressure_hazard = 0x01;
    else if (pressure > HIGH_PRESSURE_THRESHOLD)
        *pt_high_pressure_hazard = 0x01;

    /* Check for water level hazard. Note that "distance" here means the distance measured
     * by the ultrasonic sensor down to the water surface, so a larger distance means a
     * lower water level, and a smaller distance means a higher (flood-risk) water level */
    *pt_low_water_level_hazard = 0x00;
    *pt_high_water_level_hazard = 0x00;

    if (distance > LOW_WATER_LEVEL_THRESHOLD)
        *pt_low_water_level_hazard = 0x01;
    else if (distance < HIGH_WATER_LEVEL_THRESHOLD)
        *pt_high_water_level_hazard = 0x01;
}

/* Function: put the I2C (BMP180) and ultrasonic (HC-SR04) signal pins into a high-impedance
 *           (floating) state. This must be done before the sensors are powered off, otherwise
 *           the pull-up resistors / driven signal levels on these pins could leak current back
 *           into the de-energized sensors through their I/O pins, wasting power during deep sleep
 * Parameters: none
 * Return: none
 */
void isolate_sensor_signal_pins(void)
{
    /* I2C: remove pull-up so SDA/SCL float instead of sourcing current into the powered-down BMP180 */
    gpio_set_pull_mode(SDA_PIN, GPIO_FLOATING);
    gpio_set_pull_mode(SCL_PIN, GPIO_FLOATING);

    /* HC-SR04: TRIG set as floating input (instead of a driven output) so it does not leak
     * current into the powered-down sensor. ECHO is already an input, so it is left untouched */
    gpio_set_direction(TRIG_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(TRIG_PIN, GPIO_FLOATING);
}

/* Function: restore the I2C and ultrasonic sensor signal pins to their normal operating
 *           configuration (pull-ups enabled / TRIG as output), reverting what
 *           isolate_sensor_signal_pins() did, right after the sensors are powered back on
 * Parameters: none
 * Return: none
 */
void restore_sensor_signal_pins(void)
{
    /* I2C: re-enable pull-up, required for proper I2C bus operation */
    gpio_set_pull_mode(SDA_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SCL_PIN, GPIO_PULLUP_ONLY);

    /* HC-SR04: TRIG is set back to output mode, driven low (idle level, ready to trigger a measurement) */
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(TRIG_PIN, 0);
}

/* App Main Function */
void app_main(void)
{
    /* Variables which store sensor's measurements */
    float temperature = 0.0;
    uint32_t pressure = 0;
    float distance = 0.0;

    /* Variables used for hazard checking */
    char low_temperature_hazard = 0x00;
    char high_temperature_hazard = 0x00;
    char low_pressure_hazard = 0x00;
    char high_pressure_hazard = 0x00;
    char low_water_level_hazard = 0x00;
    char high_water_level_hazard = 0x00;

    /* Variable used as LoRaWAN communication parameters */
    TConfig_LoRaWAN config_lorawan;

    /* Variable used to configure GPIOs */
    gpio_config_t io_conf = {};

    /* Variables used as LoRaWAN payload buffer */
    char payload_asci_hex[60] = {0};

    /* Configure the power management module to lock the CPU clock frequency at 80MHz
     * (instead of the default, higher frequency), which reduces active-mode current
     * consumption. min_freq_mhz == max_freq_mhz pins the frequency instead of scaling it
     * dynamically, and light sleep is left disabled since deep sleep is used instead */
    esp_pm_config_t pm_config = {
        .max_freq_mhz = 80,
        .min_freq_mhz = 80,
        .light_sleep_enable = false
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));


    /* IMPORTANT:
     * In Chapter 07, the deep sleep feature is used. This means the superloop approach (an
     * infinite loop that keeps the CPU always running) is no longer suitable for saving energy.
     * Instead, app_main() runs the whole measurement/transmission cycle exactly once and then
     * puts the device into deep sleep, from which it wakes up by resetting and running
     * app_main() again from the start. The IoT project therefore follows this operation flow:
     *
     * Step 1: Init everything, including powering up peripherals
     * Step 2: Read all sensors
     * Step 3: Compose LoRaWAN payload containing all sensors' readings and hazards detected.
     * Step 4: Send payload via LoRaWAN communication
     * Step 5: Cut-off power for all peripherals (except BMP180, which is an ultra low power peripheral by default)
     *         and configure GPIOs into float or high impedance modes.
     *         Also, use the GPIO Hold feature to keep the MOSFET gate level at '0' during deep sleep.
     * Step 6: Deep sleep mode (for 1 hour)
     */

    /* ---------------------------------------------------------
     * Step 1: Init everything, include powering up peripherals
     * ---------------------------------------------------------
     */

    /* Release the GPIO hold that was set on the peripherals power-control GPIO and on the
     * on-board LED GPIO before entering the previous deep sleep cycle, so that they can be
     * driven normally again (see gpio_hold_en()/gpio_deep_sleep_hold_en() at the end of this
     * function for why the hold was set in the first place) */
    gpio_hold_dis(GPIO_TURN_PERIPHERALS_ON_OFF);
    gpio_hold_dis(GPIO_ONBOARD_LED);

    /* Configure the peripherals power-control GPIO and the on-board LED GPIO as outputs,
     * with no pull-up/pull-down and no interrupt, using the combined bit-mask defined above */
    io_conf.intr_type = GPIO_INTR_DISABLE;      // disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;            // set as output mode
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL; // bit-mask
    io_conf.pull_down_en = 0;                   // disable pull-down mode
    io_conf.pull_up_en = 0;                     // disable pull-up mode
    gpio_config(&io_conf);                      // configure GPIO with the given settings

    /* Turn all peripherals on (drives the MOSFET gate that switches their power rail),
     * and turn the onboard LED off */
    gpio_set_level(GPIO_TURN_PERIPHERALS_ON_OFF, 1);
    gpio_set_level(GPIO_ONBOARD_LED, 0);

    /* First of all, the I²C driver and the BMP180 sensor are initialized */
    i2cdev_init();
    bmp180_dev_t bmp_dev = {};
    ESP_ERROR_CHECK(bmp180_init_desc(&bmp_dev, I2C_NUM_0, SDA_PIN, SCL_PIN));
    ESP_ERROR_CHECK(bmp180_init(&bmp_dev));

    /* Then, the ultrasonic sensor (HC-SR04) is initialized */
    ultrasonic_sensor_t us_sensor = {
        .trigger_pin = TRIG_PIN,
        .echo_pin = ECHO_PIN,
    };
    ultrasonic_init(&us_sensor);

    /* Configure the LoRaWAN module: start its UART, then fill in the LoRaWAN session
     * parameters (keys, addresses, channel mask, join mode, data rate, class, etc.)
     * and send them to the module as AT commands.
     * Don't forget to replace the credentials below with your own LoRaWAN network credentials
     */
    init_uart_lorawan();
    snprintf(config_lorawan.APPSKEY, sizeof(config_lorawan.APPSKEY), "00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00");
    snprintf(config_lorawan.NWSKEY, sizeof(config_lorawan.NWSKEY), "00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00");
    snprintf(config_lorawan.APPEUI, sizeof(config_lorawan.APPEUI), "00:00:00:00:00:00:00:00");
    snprintf(config_lorawan.DEVADDR, sizeof(config_lorawan.DEVADDR), "00:00:00:00");
    snprintf(config_lorawan.CHMASK, sizeof(config_lorawan.CHMASK), "00FF:0000:0000:0000:0000:0000");
    config_lorawan.msg_sent_confirmation = LORAWAN_SENT_MSG_WITHOUT_CONFIRMATION;
    config_lorawan.join_mode = LORAWAN_JOIN_MODE_ABP;
    config_lorawan.adr = LORAWAN_ADR_OFF;
    config_lorawan.dr = LORAWAN_DR_LEVEL_2;
    config_lorawan.class = LORAWAN_CLASS_A;
    configure_lorawan(&config_lorawan);

    /* ------------------------
     * Step 2: Read all sensors
     * ------------------------
     */
    /* Turn all peripherals on and wait 50ms for them to properly power up and stabilize */
    gpio_set_level(GPIO_TURN_PERIPHERALS_ON_OFF, 1);
    gpio_set_level(GPIO_ONBOARD_LED, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Restore the BMP180 (I²C) and ultrasonic sensor communication GPIOs to their normal
     * operating state, since they were left isolated (floating) by the previous deep sleep cycle */
    restore_sensor_signal_pins();

    /* Time to read the BMP180 sensor (temperature and barometric pressure) */
    esp_err_t bmp_err = bmp180_measure(&bmp_dev, &temperature, &pressure, BMP180_MODE_STANDARD);

    // convert temperature from Celsius (as returned by the BMP180 driver) to Fahrenheit
    temperature = (temperature * 9.0 / 5.0) + 32.0;

    // convert barometric pressure from Pascal (as returned by the BMP180 driver) to Hectopascal
    pressure = pressure / 100.0;

    if (bmp_err == ESP_OK)
    {
        ESP_LOGI(TAG_LOGS_BMP180, "Temperature: %.2f F | Barometric pressure: %lu hPa", temperature, pressure);
    }
    else
    {
        ESP_LOGE(TAG_LOGS_BMP180, "Error: %s", esp_err_to_name(bmp_err));
    }

    /* Time to read the ultrasonic sensor (HC-SR04). 400 cm is the maximum distance (in cm) the
     * sensor is allowed to measure; readings beyond that are reported as an error/out-of-range */
    esp_err_t us_err = ultrasonic_measure(&us_sensor, 400, &distance);

    if (us_err == ESP_OK)
    {
        ESP_LOGI(TAG_LOGS_HCSR04, "Distance from water: %.2f m", distance);
    }
    else
    {
        ESP_LOGE(TAG_LOGS_HCSR04, "Error: %s", esp_err_to_name(us_err));
    }

    /* Time to check if readings show hazards */
    check_hazards(&low_temperature_hazard, &high_temperature_hazard,
                  &low_pressure_hazard, &high_pressure_hazard,
                  &low_water_level_hazard, &high_water_level_hazard,
                  temperature, pressure, distance);

    /* -------------------------------------------------------------------------------------
     * Step 3: Compose LoRaWAN payload containing all sensors' readings and hazards detected
     * -------------------------------------------------------------------------------------
     */                  
    /* Time to compose the LoRaWAN payload, encoded as an HEX-ASCII string, where each pair of
     * hex characters represents one byte. The payload fields, in order, are:
     *   - low temperature hazard flag    (1 byte / 2 hex chars)
     *   - high temperature hazard flag   (1 byte / 2 hex chars)
     *   - low pressure hazard flag       (1 byte / 2 hex chars)
     *   - high pressure hazard flag      (1 byte / 2 hex chars)
     *   - low water level hazard flag    (1 byte / 2 hex chars)
     *   - high water level hazard flag   (1 byte / 2 hex chars)
     *   - temperature, truncated to an integer in Fahrenheit (1 byte / 2 hex chars)
     *   - barometric pressure, truncated to an integer in hPa (2 bytes / 4 hex chars)
     *   - distance to water surface, truncated to an integer in meters (1 byte / 2 hex chars)
     * Truncating temperature, pressure and distance to integers keeps the payload small,
     * which reduces on-air time and therefore energy consumption during LoRaWAN transmission */
    snprintf(payload_asci_hex, sizeof(payload_asci_hex), "%02X%02X%02X%02X%02X%02X%02X%04X%02X", low_temperature_hazard, high_temperature_hazard,
             low_pressure_hazard, high_pressure_hazard,
             low_water_level_hazard, high_water_level_hazard,
             (uint8_t)temperature, (uint16_t)pressure, (uint8_t)distance);


    /* ----------------------------------------------
     * Step 4: Send payload via LoRaWAN communication
     * ----------------------------------------------
     */   
    send_LoRaWAN_payload(payload_asci_hex);

    /* -----------------------------------------------------------------------------------------------------------
     * Step 5: Cut-off power for all peripherals (except BMP180, which is an ultra low pwer peripheral by default)
     *         and configure GPIOs into float or high impedance modes.
     *         Also, use GPIO Hold feature to keep MOSFET gate level in '0'.
     * -----------------------------------------------------------------------------------------------------------
     */ 
    isolate_sensor_signal_pins();
    gpio_set_level(GPIO_TURN_PERIPHERALS_ON_OFF, 0);   // cut power to the MOSFET-switched peripherals (ultrasonic sensor)
    gpio_set_level(GPIO_ONBOARD_LED, 0);                // turn onboard LED off

    /* Deep sleep resets the GPIO configuration by default, which would let the peripherals
     * power-control GPIO and the LED GPIO float or glitch high during the transition.
     * GPIO Hold "freezes" the current output level of these pins so they stay at '0'
     * even during deep sleep, and gpio_deep_sleep_hold_en() makes that hold persist
     * throughout deep sleep (it is normally released automatically on wake-up/reset) */
    gpio_hold_en(GPIO_TURN_PERIPHERALS_ON_OFF);
    gpio_hold_en(GPIO_ONBOARD_LED);
    gpio_deep_sleep_hold_en();

    /* ----------------------------------------------
     * Step 6:  Deep sleep mode (for 1 hour)
     * ----------------------------------------------
     */
     /* The operation flow is now complete. Enter deep sleep for one hour; the ESP32 will reset
      * and run app_main() again from the beginning once the timer wakeup fires.
      * The two lines below are left commented out as a reference: they would additionally power
      * down the RTC peripherals and the crystal oscillator domains for extra energy savings,
      * at the cost of the corresponding peripherals not being available during deep sleep */
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_DURATION * DEEP_SLEEP_US_TOS_FACTOR);
    esp_deep_sleep_start();
}
