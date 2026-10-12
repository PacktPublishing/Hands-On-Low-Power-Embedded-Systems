/* Header file: definitions, configuration struct and prototypes used to communicate with an
 * external LoRaWAN module (RAK811-like, controlled through AT commands) over UART.
 * This module is used by the IoT project to send sensor readings and hazard flags to a
 * LoRaWAN network, so the values here mirror the AT command set of the LoRaWAN module. */
#ifndef LORAWAN_DEFS_H
#define LORAWAN_DEFS_H

/* Definition: GPIOs used for UART communication between the ESP32 and the LoRaWAN module */
#define LORAWAN_TEST_TXD       22
#define LORAWAN_TEST_RXD       23

/* Definition: AT command buffer max size (in bytes), used to size the buffers that hold
 * the AT command strings sent to the LoRaWAN module */
#define TAM_MAX_CMD_AT                   150

/* Definition: maximum time to wait, in milliseconds, for the LoRaWAN module's response
 * ('\n'-terminated line) after sending an AT command. Some commands answer only after a
 * radio operation ends (e.g. AT+SENDB answers AT_TX_OK after the transmission), which may
 * take up to 10 s in the worst case. The wait ends as soon as the response arrives, so this
 * limit costs nothing when the module answers quickly; it is only reached if it never does */
#define AT_RESPONSE_TIMEOUT_MS   10000  //ms

/* Definition: maximum time to wait, in milliseconds, for the response to a configuration
 * command (AT+CHMASK, AT+NJM, etc.). These commands don't involve any radio operation, so the
 * module answers them in less than 1 s; if it doesn't, the command is re-sent */
#define AT_CONFIG_RESPONSE_TIMEOUT_MS   2000  //ms

/* Definition: maximum number of times an AT command is sent when the module doesn't answer it */
#define AT_CMD_MAX_ATTEMPTS      5

/* Definitions: the LoRaWAN module sends a boot banner (e.g. "Radioenge AT 1.3.35B0") when it
 * finishes booting; no command is sent before that, or up to LORAWAN_BOOT_TIMEOUT_MS after the
 * UART is initialized, if the banner is not received */
#define LORAWAN_BOOT_BANNER      "Radioenge"
#define LORAWAN_BOOT_TIMEOUT_MS  3000  //ms

/* Definitions - message confirmation: whether the LoRaWAN module should require a network
 * acknowledgement (confirmed uplink) for sent messages, used as the value of the AT+CFM command */
#define LORAWAN_SENT_MSG_WITH_CONFIRMATION       '1'
#define LORAWAN_SENT_MSG_WITHOUT_CONFIRMATION    '0'

/* Definitions - Join mode: how the device joins the LoRaWAN network, used as the value of
 * the AT+NJM command. ABP (Activation By Personalization) uses fixed session keys/address,
 * while OTAA (Over-The-Air Activation) negotiates them dynamically with the network server */
#define LORAWAN_JOIN_MODE_ABP             '0'
#define LORAWAN_JOIN_MODE_OTAA            '1'

/* Definitions - ADR (Adaptive Data Rate): lets the network server automatically adjust the
 * device's data rate and transmit power, used as the value of the AT+ADR command */
#define LORAWAN_ADR_OFF            '0'
#define LORAWAN_ADR_ON             '1'

/* Definitions - DR (Data Rate) levels, used as the value of the AT+DR command. Higher levels
 * mean faster, shorter-range transmissions (less time on air); lower levels mean slower,
 * longer-range transmissions (more time on air, more energy spent per message) */
#define LORAWAN_DR_LEVEL_0                '0'
#define LORAWAN_DR_LEVEL_1                '1'
#define LORAWAN_DR_LEVEL_2                '2'
#define LORAWAN_DR_LEVEL_3                '3'
#define LORAWAN_DR_LEVEL_4                '4'
#define LORAWAN_DR_LEVEL_5                '5'
#define LORAWAN_DR_LEVEL_6                '6'

/* Definitions - LoRaWAN Class, used as the value of the AT+CLASS command.
 * Class A is the default, lowest-power class (only listens for downlink right after an
 * uplink); Class C keeps the receiver open almost continuously (higher power consumption) */
#define LORAWAN_CLASS_A                  'A'
#define LORAWAN_CLASS_C                  'C'


/* LoRaWAN communication config struct: holds every parameter needed to configure the
 * LoRaWAN module (session keys, addresses, channel mask and radio/session settings)
 * before starting to send data over the network */
typedef struct __attribute__((__packed__))
{
    /* LoRaWAN credentials and channel mask */
    char APPSKEY[60];  // Application Session Key (ABP), as a colon-separated hex string
    char NWSKEY[60];   // Network Session Key (ABP), as a colon-separated hex string
    char APPEUI[30];   // Application EUI, as a colon-separated hex string
    char DEVADDR[15];  // Device Address, as a colon-separated hex string
    char CHMASK[35];   // Channel mask, selecting which LoRaWAN channels are enabled

    /* Message sent confirmation flag: one of LORAWAN_SENT_MSG_WITH/WITHOUT_CONFIRMATION */
    char msg_sent_confirmation;

    /* Join mode: one of LORAWAN_JOIN_MODE_ABP / LORAWAN_JOIN_MODE_OTAA */
    char join_mode;

    /* ADR: one of LORAWAN_ADR_OFF / LORAWAN_ADR_ON */
    char adr;

    /* DR: one of the LORAWAN_DR_LEVEL_x values */
    char dr;

    /* LoRaWAN class: one of LORAWAN_CLASS_A / LORAWAN_CLASS_C */
    char class;
}TConfig_LoRaWAN;

/* Prototypes */
void init_uart_lorawan(void);
void configure_lorawan(TConfig_LoRaWAN *pt_lorawan);
void send_LoRaWAN_payload(char *pt_payload);

#endif