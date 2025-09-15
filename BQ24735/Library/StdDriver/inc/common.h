#ifndef COMMON_H
#define COMMON_H

/************************************************************************
 * Include Header Files
 ************************************************************************/
#include "Function_define_MS51_16K.h"

/****************************************************************************
 * MACROS
 ***************************************************************************/
#define C_BINARY_U8           2u
#define C_DECIMAL_U8         10u
#define C_HEX_U8             16u
#define HIGH 1
#define LOW  0


#define  util_GetBitMask(bit)          (1<<(bit))
#define  util_BitSet(x,bit)            ((x) |=  util_GetBitMask(bit))
#define  util_BitClear(x,bit)          ((x) &= ~util_GetBitMask(bit))
#define  util_BitToggle(x,bit)         ((x) ^=  util_GetBitMask(bit))
#define  util_UpdateBit(x,bit,val)     ((val)? util_BitSet(x,bit): util_BitClear(x,bit))


#define  util_GetBitStatus(x,bit)      (((x)&(util_GetBitMask(bit)))!=0u)
#define  util_IsBitSet(x,bit)          (((x)&(util_GetBitMask(bit)))!=0u)
#define  util_IsBitCleared(x,bit)      (((x)&(util_GetBitMask(bit)))==0u)


#define  util_AreAllBitsSet(x,BitMask) (((x)&(BitMask))==BitMask)
#define  util_AreAnyBitsSet(x,BitMask) (((x)&(BitMask))!=0x00u)

/***************************************************************************************************
                          Macros for Dec2Ascii,Hec2Ascii and Acsii2Hex conversion
 ****************************************************************************************************/
#define util_Dec2Ascii(Dec)	 ((Dec)+0x30)
#define util_Hex2Ascii(Hex) (((Hex)>0x09) ? ((Hex) + 0x37): ((Hex) + 0x30)) 
#define util_Ascii2Hex(Asc) (((Asc)>0x39) ? ((Asc) - 0x37): ((Asc) - 0x30))
/***************************************************************************************************/

/***************************************************************************************************
                             Macros to find the mod of a number
 ***************************************************************************************************/
#define util_GetMod8(dividend,divisor)  (uint8_t) (dividend - (divisor * (uint8_t) (dividend/divisor)))
#define util_GetMod16(dividend,divisor) (uint16_t)(dividend - (divisor * (uint16_t)(dividend/divisor)))
#define util_GetMod32(dividend,divisor) (uint32_t)(dividend - (divisor * (uint32_t)(dividend/divisor)))
/***************************************************************************************************/

/****************************************************************************
 * FUNCTIONS PROTOTYPES
 ****************************************************************************/
unsigned char Read_APROM_BYTE(unsigned int code *u16_addr);


#endif