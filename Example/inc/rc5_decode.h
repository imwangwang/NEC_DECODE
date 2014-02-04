#ifndef __RC5_DECODE_H
#define __RC5_DECODE_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_conf.h"
#include "ir_decode.h"

typedef struct
{   
  __IO uint8_t FieldBit;   /*!< Field bit */   
  __IO uint8_t ToggleBit;  /*!< Toggle bit field */
  __IO uint8_t Address;    /*!< Address field */ 
  __IO uint8_t Command;    /*!< Command field */ 
  
} RC5_Frame_TypeDef;

typedef struct
{
 __IO uint16_t data;     /*!< RC5 data */ 
 __IO uint8_t  status;   /*!< RC5 status */
 __IO uint8_t  lastBit;  /*!< RC5 last bit */
 __IO uint8_t  bitCount; /*!< RC5 bit count */
} tRC5_packet;

enum RC5_lastBitType
{
  RC5_ZER,
  RC5_ONE,
  RC5_NAN,
  RC5_INV
};

typedef enum RC5_lastBitType tRC5_lastBitType;

#define RC5_1T_TIME                          0x00
#define RC5_2T_TIME                          0x01
#define RC5_WRONG_TIME                       0xFF
#define RC5_TIME_OUT_US                      3600
#define RC5_T_US                             900     /*!< Half bit period */
#define RC5_T_TOLERANCE_US                   200    /*!< Tolerance time */
#define RC5_NUMBER_OF_VALID_PULSE_LENGTH     2
#define RC5_PACKET_BIT_COUNT                 13      /*!< Total bits */

#define RC5_PACKET_STATUS_EMPTY              1 << 0

void RC5_DeInit(void);
void RC5_Init(void);
void RC5_Decode(RC5_Frame_TypeDef *rc5_frame);
void RC5_ResetPacket(void);
void RC5_DataSampling(uint32_t rawPulseLength, uint32_t edge);
char* RC5_GetDevice(uint8_t address);
char* RC5_GetCommand(uint8_t command);
extern void RC5_ReceiveInterrupt(void);

#ifdef __cplusplus
}
#endif

#endif
