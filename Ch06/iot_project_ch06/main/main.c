/* Program: this is the source code for IoT project of Chapter 06 of "Hands-On Low-Power Embedded Systems" book
 *          This has hardware optimizations.
 *
 * Author: Pedro Bertoleti
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

/* Header files for needed libraries */
#include "bmp180.h"
#include "ultrasonic.h"

/* Header file for LoRaWAN communication */
#include "lorawan.h"

/* GPIOs used for I²C communication with BMP180 sensor */
#define SDA_PIN     18
#define SCL_PIN     19

/* GPIOs used for communication with ultrasonic (HC-SR04) sensor */
#define TRIG_PIN    5
#define ECHO_PIN    21

/* GPIO used to turn peripherals on/off (to save energy) */
#define GPIO_TURN_PERIPHERALS_ON_OFF      17

/* GPIO used to control onboard LED */
#define GPIO_ONBOARD_LED                  2

/* GPIO mask */
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_TURN_PERIPHERALS_ON_OFF) | (1ULL<<GPIO_ONBOARD_LED))

/* Thresholds for hazards */
/* Temperature (Fahrenheit) */
#define LOW_TEMP_THRESHOLD     23.0
#define HIGH_TEMP_THRESHOLD    95.0

/* Barometric pressure (Hectopascal) */
#define LOW_PRESSURE_THRESHOLD     940.0
#define HIGH_PRESSURE_THRESHOLD    970.0

/* Water level distance (m) */
#define LOW_WATER_LEVEL_THRESHOLD     12.0
#define HIGH_WATER_LEVEL_THRESHOLD    10.0

/* Debug tags */
static const char *TAG_LOGS_BMP180 = "BMP180";
static const char *TAG_LOGS_HCSR04 = "HC-SR04";

/* Function prototypes */
void check_hazards(char * pt_low_temp_hazard, char * pt_high_temp_hazard,
                   char * pt_low_pressure_hazard, char * pt_high_pressure_hazard, 
                   char * pt_low_water_level_hazard, char * pt_high_water_level_hazard, 
                   float temp, uint32_t pressure, float distance);
void isolate_sensor_signal_pins(void);                   

/* Function: check if current values of temperature, pressure and water level indicate
 *           hazard conditions 
 * Parameters: - hazard indicators (low/high teperature, barometric pressure and water level)
 *             - current readings of temperature, barometric pressure and water level
 * Return: none 
 */
void check_hazards(char * pt_low_temp_hazard, char * pt_high_temp_hazard,
                   char * pt_low_pressure_hazard, char * pt_high_pressure_hazard, 
                   char * pt_low_water_level_hazard, char * pt_high_water_level_hazard, 
                   float temp, uint32_t pressure, float distance)
{
    /* Check for temperature hazzard */
    *pt_low_temp_hazard = 0x00;
    *pt_high_temp_hazard = 0x00;

    if (temp < LOW_TEMP_THRESHOLD)
        *pt_low_temp_hazard = 0x01;
    else if (temp > HIGH_TEMP_THRESHOLD)
        *pt_high_temp_hazard = 0x01;

    /* Check for barometric pressure hazzard */
    *pt_low_pressure_hazard = 0x00;
    *pt_high_pressure_hazard = 0x00;

    if (pressure < LOW_PRESSURE_THRESHOLD)
        *pt_low_pressure_hazard = 0x01;
    else if (pressure > HIGH_PRESSURE_THRESHOLD)
        *pt_high_pressure_hazard = 0x01;    

    /* Check for water level hazzard */
    *pt_low_water_level_hazard = 0x00;
    *pt_high_water_level_hazard = 0x00;

    if (distance > LOW_WATER_LEVEL_THRESHOLD)
        *pt_low_water_level_hazard = 0x01;
    else if (distance < HIGH_WATER_LEVEL_THRESHOLD)
        *pt_high_water_level_hazard = 0x01;        
}

/* Function: put I2C and ultrasonic signal pins in high-impedance state
 *           to avoid leakage current into de-energized sensors 
 * Parameters: none
 * Return: none
 */
void isolate_sensor_signal_pins(void)
{
    /* I2C: remove pull-up */
    gpio_set_pull_mode(SDA_PIN, GPIO_FLOATING);
    gpio_set_pull_mode(SCL_PIN, GPIO_FLOATING);

    /* HC-SR04: TRIG set as floating GPIO, ECHO is set as input */
    gpio_set_direction(TRIG_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(TRIG_PIN, GPIO_FLOATING);
}

/* Function: restore pull-ups to comunication GPIOs
 * Parameters: none
 * Return: none
 */
void restore_sensor_signal_pins(void)
{
    /* I2C: activate pull-up */
    gpio_set_pull_mode(SDA_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SCL_PIN, GPIO_PULLUP_ONLY);

    /* HC-SR04: TRIG is set as output again */
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

    /* Configure GPIO to turn peripherals on/off */     
    io_conf.intr_type = GPIO_INTR_DISABLE;  //disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT; //set as output mode
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;  //bit-mask
    io_conf.pull_down_en = 0; //disable pull-down mode
    io_conf.pull_up_en = 0; //disable pull-up mode
    gpio_config(&io_conf); //configure GPIO with the given settings

    /* Turn all peripherals on, and turn onboard LED off */
    gpio_set_level(GPIO_TURN_PERIPHERALS_ON_OFF, 1);
    gpio_set_level(GPIO_ONBOARD_LED, 0);

    /* First of all, the I²C interface and BMP180 sensor is initiated */
    i2cdev_init();
    bmp180_dev_t bmp_dev = {};
    ESP_ERROR_CHECK(bmp180_init_desc(&bmp_dev, I2C_NUM_0, SDA_PIN, SCL_PIN));
    ESP_ERROR_CHECK(bmp180_init(&bmp_dev));

    /* Then, the ultrasonic sensor (HC-SR04) is initiated */
    ultrasonic_sensor_t us_sensor = {
        .trigger_pin = TRIG_PIN,
        .echo_pin    = ECHO_PIN,
    };
    ultrasonic_init(&us_sensor);

    /* Configure LoRaWAN module 
     * Don't forget to replace credentials with your own
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

    /* With all peripherals initiated, now it's time to go to the main loop
     * (and stay there "forever")
     */
    while (1) {
        /* Turn all peripherals on for a cycle and wait 50ms for them to properly start*/
        gpio_set_level(GPIO_TURN_PERIPHERALS_ON_OFF, 1);
        gpio_set_level(GPIO_ONBOARD_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(50)); 

        /* Re-start BMP180 (I²C) and ultrassonic sensors, once, their communication GPIOs have been isolated in previous cycle */
        restore_sensor_signal_pins();

        /* Time to read BMP180 sensor */        
        esp_err_t bmp_err = bmp180_measure(&bmp_dev, &temperature, &pressure, BMP180_MODE_STANDARD);
        
        //convert temperature from Celsius to Fahrenheit
        temperature = (temperature * 9.0 / 5.0) + 32.0;  

        //convert barometic pressure from Pascal to Hectopascal
        pressure = pressure / 100.0;
        
        if (bmp_err == ESP_OK) 
        {
            ESP_LOGI(TAG_LOGS_BMP180, "Temperature: %.2f F | Barometric pressure: %lu hPa", temperature, pressure);
        } 
        else 
        {
            ESP_LOGI(TAG_LOGS_BMP180, "Error: %s", esp_err_to_name(bmp_err));
        }

        /* Time to read ultrasonic sensor (HC-SR04) */
        esp_err_t us_err = ultrasonic_measure(&us_sensor, 400, &distance);
 
        if (us_err == ESP_OK) 
        {
            ESP_LOGI(TAG_LOGS_HCSR04, "Distance from water: %.2f m", distance);
        } 
        else 
        {
            ESP_LOGI(TAG_LOGS_HCSR04, "Error: %s", esp_err_to_name(us_err));
        }

        /* Time to check if readings show hazards */
        check_hazards(&low_temperature_hazard, &high_temperature_hazard,
                      &low_pressure_hazard, &high_pressure_hazard, 
                      &low_water_level_hazard, &high_water_level_hazard,
                      temperature, pressure, distance);

        /* Time to compose LoRaWAN payload (HEX-ASCII format) and send it using LoRaWAN communication. 
         * The temperature is send x100 (to fit in a single byte), and distance is sent in */
        snprintf(payload_asci_hex, sizeof(payload_asci_hex),  "%02X%02X%02X%02X%02X%02X%02X%04X%02X", low_temperature_hazard, high_temperature_hazard, 
                                                                                                      low_pressure_hazard, high_pressure_hazard, 
                                                                                                      low_water_level_hazard, high_water_level_hazard, 
                                                                                                      (uint8_t)temperature, (uint16_t)pressure, (uint8_t)distance);


        send_LoRaWAN_payload(payload_asci_hex);

        /* Isolate GPIOs used for sensors communication, in order to avoid parasite current consumption */
        isolate_sensor_signal_pins();
        
        /* The cycle is now complete. Now, let's turn all peripherals off and
         * wait for an hour (3600000 ms) to the next read-send cycle 
         */
        gpio_set_level(GPIO_TURN_PERIPHERALS_ON_OFF, 0);
        gpio_set_level(GPIO_ONBOARD_LED, 0);
        vTaskDelay(pdMS_TO_TICKS(3600000));        
    }
}
