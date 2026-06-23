#include <string.h>
#include <esp_task_wdt.h>
#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "lorawan.h"

/* Configurations for UART communication with LoRaWAN module  */
#define LORAWAN_TEST_TXD       22
#define LORAWAN_TEST_RXD       23
#define LORAWAN_TEST_RTS       (UART_PIN_NO_CHANGE)
#define LORAWAN_TEST_CTS       (UART_PIN_NO_CHANGE)
#define LORAWAN_UART_BAUD_RATE 115200
#define LORAWAN_UART_PORT_NUM  1

/* Uart buffer size */
#define BUF_SIZE (512)

/* RX UART buffer */
static char buffer_recepcao[BUF_SIZE] = {0};

/* Debug tag */
static const char *TAG_LOGS_LORAWAN = "LORAWAN";

/* Function: send AT command to LoRaWAN module using UART interface
 * Parameters: command
 * Return: none
 */
void send_command_over_UART(char *pt_cmd, int tamanho)
{
    int tam = 0;
    bool is_lorawan_module_busy = false;

    do
    {
        uart_write_bytes(LORAWAN_UART_PORT_NUM, pt_cmd, tamanho);
        vTaskDelay(pdMS_TO_TICKS(TIME_BETWEEN_AT_CMDS));        

        memset(buffer_recepcao, 0x00, BUF_SIZE);
        tam = uart_read_bytes(LORAWAN_UART_PORT_NUM,
                              (char *)buffer_recepcao, (BUF_SIZE - 1),
                              TIME_BETWEEN_AT_CMDS / portTICK_PERIOD_MS);

        /* Check if LoRaWAN module is busy */
        if (strstr(buffer_recepcao, "BUSY") != NULL)
        {
            ESP_LOGE(TAG_LOGS_LORAWAN, "LoRaWAN module is BUSY: %s. Command will be re-sent in 5 seconds from now...", buffer_recepcao);
            vTaskDelay(pdMS_TO_TICKS(5000));
            
            is_lorawan_module_busy = true;
        }
        else
        {
            is_lorawan_module_busy = false;
        }
    } while (is_lorawan_module_busy == true);

    if (tam > 0)
    {
        ESP_LOGI(TAG_LOGS_LORAWAN, "String received from LoRaWAN module: %s", buffer_recepcao);
    }    
}

/* Function: inicializa UART de comunicação com módulo LoRaWAN
 * Parameters: none
 * Return: none
 */
void init_uart_lorawan(void)
{
    /* Configure UART */
    uart_config_t uart_config = {
        .baud_rate = LORAWAN_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(LORAWAN_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(LORAWAN_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LORAWAN_UART_PORT_NUM, LORAWAN_TEST_TXD, LORAWAN_TEST_RXD, LORAWAN_TEST_RTS, LORAWAN_TEST_CTS));
}

/* Function: configure LoRaWAN module
 * Parameters: configuration struct
 * Return: none
 */
void configure_lorawan(TConfig_LoRaWAN *pt_lorawan)
{
    char cmd_at[TAM_MAX_CMD_AT] = {0};    

    /* Configure channel mask */
    ESP_LOGI(TAG_LOGS_LORAWAN, "Configure channel mask: %s ...", pt_lorawan->CHMASK);
    memset(cmd_at, 0x00, sizeof(cmd_at));
    snprintf(cmd_at, sizeof(cmd_at), "AT+CHMASK=%s\n\r", pt_lorawan->CHMASK);
    send_command_over_UART(cmd_at, strlen(cmd_at));   

    /* Configure Join Mode */
    ESP_LOGI(TAG_LOGS_LORAWAN, "Configure Join Mode ...");
    memset(cmd_at, 0x00, sizeof(cmd_at));
    snprintf(cmd_at, sizeof(cmd_at), "AT+NJM=%c\n\r", pt_lorawan->join_mode);
    send_command_over_UART(cmd_at, strlen(cmd_at));    

    /* Configure LoRaWAN Device Address */
    ESP_LOGI(TAG_LOGS_LORAWAN, "Configure LoRaWAN Device Address: %s ...", pt_lorawan->DEVADDR);
    memset(cmd_at, 0x00, sizeof(cmd_at));
    snprintf(cmd_at, sizeof(cmd_at), "AT+DADDR=%s\n\r", pt_lorawan->DEVADDR);
    send_command_over_UART(cmd_at, strlen(cmd_at));    

    /* Configure LoRaWAN Application EUI */
    ESP_LOGI(TAG_LOGS_LORAWAN, "Configure LoRaWAN Application EUI: %s ...", pt_lorawan->APPEUI);
    memset(cmd_at, 0x00, sizeof(cmd_at));
    snprintf(cmd_at, sizeof(cmd_at), "AT+APPEUI=%s\n\r", pt_lorawan->APPEUI);
    send_command_over_UART(cmd_at, strlen(cmd_at));    

    /* Configure LoRaWAN Application Session Key */
    ESP_LOGI(TAG_LOGS_LORAWAN, "Configure LoRaWAN Application Session Key: %s ...", pt_lorawan->APPSKEY);
    memset(cmd_at, 0x00, sizeof(cmd_at));
    snprintf(cmd_at, sizeof(cmd_at), "AT+APPSKEY=%s\n\r", pt_lorawan->APPSKEY);
    send_command_over_UART(cmd_at, strlen(cmd_at));    

    /* Configure LoRaWAN Network Session Key */
    ESP_LOGI(TAG_LOGS_LORAWAN, "Configure LoRaWAN Network Session Key: %s ...", pt_lorawan->NWSKEY);
    memset(cmd_at, 0x00, sizeof(cmd_at));
    snprintf(cmd_at, sizeof(cmd_at), "AT+NWKSKEY=%s\n\r", pt_lorawan->NWSKEY);
    send_command_over_UART(cmd_at, strlen(cmd_at));    

    /* Configure LoRaWAN ADR */
    ESP_LOGI(TAG_LOGS_LORAWAN, "Configure LoRaWAN ADR: %c ...", pt_lorawan->adr);
    memset(cmd_at, 0x00, sizeof(cmd_at));
    snprintf(cmd_at, sizeof(cmd_at), "AT+ADR=%c\n\r", pt_lorawan->adr);
    send_command_over_UART(cmd_at, strlen(cmd_at));    

    /* Configure LoRaWAN Data Rate */
    ESP_LOGI(TAG_LOGS_LORAWAN, "Configure LoRaWAN Data Rate: %c ...", pt_lorawan->dr);
    memset(cmd_at, 0x00, sizeof(cmd_at));
    snprintf(cmd_at, sizeof(cmd_at), "AT+DR=%c\n\r", pt_lorawan->dr);
    send_command_over_UART(cmd_at, strlen(cmd_at));    

    /* Configure LoRaWAN Class */
    ESP_LOGI(TAG_LOGS_LORAWAN, "Configure LoRaWAN Class: %c ...", pt_lorawan->class);
    memset(cmd_at, 0x00, sizeof(cmd_at));
    snprintf(cmd_at, sizeof(cmd_at), "AT+CLASS=%c\n\r", pt_lorawan->class);
    send_command_over_UART(cmd_at, strlen(cmd_at));    

    /* Configure LoRaWAN message confirmation */
    ESP_LOGI(TAG_LOGS_LORAWAN, "Configure LoRaWAN message confirmation: %c ...", pt_lorawan->msg_sent_confirmation);
    memset(cmd_at, 0x00, sizeof(cmd_at));
    snprintf(cmd_at, sizeof(cmd_at), "AT+CFM=%c\n\r", pt_lorawan->msg_sent_confirmation);
    send_command_over_UART(cmd_at, strlen(cmd_at));    

    ESP_LOGI(TAG_LOGS_LORAWAN, "all set!");
}

/* Function: send LoRaWAN payload
 * Parameters: payload to be sent over LoRaWAN
 * Return: none
 */
void send_LoRaWAN_payload(char *pt_payload)
{
    char cmd_envio_payload[TAM_MAX_CMD_AT] = {0};

    snprintf(cmd_envio_payload, sizeof(cmd_envio_payload), "AT+SENDB=5:%s", pt_payload);
    ESP_LOGI(TAG_LOGS_LORAWAN, "AT command: %s", cmd_envio_payload);
    send_command_over_UART(cmd_envio_payload, strlen(cmd_envio_payload));
    
}