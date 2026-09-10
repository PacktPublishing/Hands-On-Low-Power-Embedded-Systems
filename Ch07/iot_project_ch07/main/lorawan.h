#ifndef LORAWAN_DEFS_H
#define LORAWAN_DEFS_H

/* Definition - AT command max size */
#define TAM_MAX_CMD_AT                   150

/* Definition: time between two AT commands send operation */
#define TIME_BETWEEN_AT_CMDS     1000  //ms

/* Definitions - message confirmation */
#define LORAWAN_SENT_MSG_WITH_CONFIRMATION       '1'
#define LORAWAN_SENT_MSG_WITHOUT_CONFIRMATION    '0'

/* Definitions - Join mode */
#define LORAWAN_JOIN_MODE_ABP             '0'
#define LORAWAN_JOIN_MODE_OTAA            '1'

/* Definitions - ADR */
#define LORAWAN_ADR_OFF            '0'
#define LORAWAN_ADR_ON             '1'

/* Definitions - ADR */
#define LORAWAN_DR_LEVEL_0                '0' 
#define LORAWAN_DR_LEVEL_1                '1'
#define LORAWAN_DR_LEVEL_2                '2'
#define LORAWAN_DR_LEVEL_3                '3'
#define LORAWAN_DR_LEVEL_4                '4'
#define LORAWAN_DR_LEVEL_5                '5'
#define LORAWAN_DR_LEVEL_6                '6' 

/* Definitions - LoRaWAN Class */
#define LORAWAN_CLASS_A                  'A'
#define LORAWAN_CLASS_C                  'C'


/* LoRaWAN communication config struct */
typedef struct __attribute__((__packed__))
{
    /* LoRaWAN credentials and masks */
    char APPSKEY[60];
    char NWSKEY[60];
    char APPEUI[30];
    char DEVADDR[15];
    char CHMASK[35];

    /* Message sent confirmation flag */
    char msg_sent_confirmation;

    /* Join mode */
    char join_mode;

    /* ADR */
    char adr;

    /* DR */
    char dr;

    /* LoRaWAN class */
    char class;
}TConfig_LoRaWAN;

#endif

/* Prototypes */
void init_uart_lorawan(void);
void configure_lorawan(TConfig_LoRaWAN *pt_lorawan);
void send_LoRaWAN_payload(char *pt_payload);