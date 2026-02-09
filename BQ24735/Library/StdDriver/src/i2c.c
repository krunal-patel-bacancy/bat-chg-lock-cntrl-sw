/**
 ******************************************************************************
 *
 * @author     Bacancy Systems LLP 
 * @date       13, September, 2021
 * @File_name: i2c.c
 ******************************************************************************
 */
 
 
 //File Function: MS51FB9AE I2C  code, the Slave address = 0x3C
 /************************************************************************
 * INCLUDES
 ************************************************************************/
#include "MS51_16K.h"
#include "Common.h"
#include "i2c.h"
#include "Function_define_MS51_16K.h"

/****************************************************************************
 * MACROS
 ***************************************************************************/
#define I2C_CLOCK                 14 
#define timeout_count             1000
#define I2C_SLAVE_ADDRESS         0x12

/****************************************************************************
 * PRIVATE FUNCTIONS
 ****************************************************************************/
void I2C_Master_Open(unsigned long u32SYSCLK, unsigned long u32I2CCLK)
{
    SFRS = 0x00;
    I2CLK = (u32SYSCLK/4/u32I2CCLK-1); 
    set_I2CON_I2CEN;

}

void I2C_Slave_Open( unsigned char u8SlaveAddress0)
{
        SFRS = 0; 
        I2ADDR = u8SlaveAddress0; 
        set_I2CON_I2CEN;
        set_I2CON_AA;
}

void I2C_Close(void)
{
    SFRS = 0;
    clr_I2CON_I2CEN;
}


void I2C_Interrupt(unsigned char u8I2CStatus)
{
    SFRS = 0;
         switch (u8I2CStatus)
         {
           case Enable: ENABLE_I2C_INTERRUPT; break;
           case Disable: DISABLE_I2C_INTERRUPT; break;
         }
}


unsigned char I2C_GetStatus(void)
{
    unsigned char u8i2cstat;
    SFRS = 0;
     u8i2cstat=I2STAT;
    return (u8i2cstat);
}

void I2C_Timeout( unsigned char u8I2CTRStatus )
{
        switch (u8I2CTRStatus)
        {
          case Enable: set_I2TOC_DIV; set_I2TOC_I2TOCEN; break;
          case Disable: clr_I2TOC_I2TOCEN; break;
        }
}


void I2C_ClearTimeoutFlag(void)
{
    SFRS = 0;
    I2TOC&=0xFE; ;
}

void I2C0_SI_Check(void)
{
    clr_I2CON_SI;
    
    while(I2CON&SET_BIT3)     /* while SI==0; */
    {
        if(I2STAT == 0x00)
        {
            set_I2CON_STO;
        }
        SI = 0;
        if(!SI)
        {
            clr_I2CON_I2CEN;
            set_I2CON_I2CEN;
            clr_I2CON_SI;
        } 
    }
}


void Init_I2C(void)
{
   P13_OPENDRAIN_MODE;          // Modify SCL pin to Open drain mode. don't forget the pull high resister in circuit
   P14_OPENDRAIN_MODE;          // Modify SDA pin to Open drain mode. don't forget the pull high resister in circuit
    /* Set I2C clock rate */
    I2CLK = I2C_CLOCK; 
    I2CPX=0; 
    /* Enable I2C */
    set_I2CON_I2CEN;                                   
}
//========================================================================================================
void I2C_Error(void)
{

	while (1);    
}
void I2C_start(void)
{
    signed int time = timeout_count;
            set_STA;                                  
            clr_SI;
            while((SI == 0) && (time > 0))
            {
            time--;
            };   
      
}
void I2C_stop(void)
{
   
		signed int t = timeout_count;
    clr_SI;
    set_STO;
    while((STO == 1) && (t > 0))
    {
        t--;
    };  
}
//========================================================================================================

void I2C_write(UINT8 u8DAT)
{                              
		UINT16 t;
		t = timeout_count;
     
    clr_STA; 
   	I2DAT = u8DAT;
    clr_SI;
    while((SI == 0) && (t > 0))
    {
        t--;
    }; 	
}
unsigned char I2C_read(unsigned char ack_mode)
{
            int t = timeout_count;
            unsigned char value = 0x00;
            set_AA;                                    
            clr_SI;
            while((SI == 0) && (t > 0))
            {
            t--;
            };           
            value = I2DAT;
            if(ack_mode == I2C_NACK)
            {
            t = timeout_count;
            clr_AA; 
            clr_SI;
            while((SI == 0) && (t > 0))
            {
            t--;
            };           
            }

            return value;
}

/****************************************************************************** 
 * Function Name : BQ24735_write
 * Description   : Write 16-bit value to BQ24735 register
 *                 BQ24735 expects: LSB first, then MSB
 * Arguments     : address - Register address (0x14, 0x15, 0x3F, etc.)
 *                 value - 16-bit value to write
 * Return Value  : None
 ******************************************************************************/
void BQ24735_write(unsigned char address, unsigned int value)
{
    I2C_start();
    I2C_write(I2C_SLAVE_ADDRESS);
    I2C_write(address);
    I2C_write(value & 0xFF);       // Send LSB first
    I2C_write((value >> 8) & 0xFF); // Send MSB second
    I2C_stop();
}

/****************************************************************************** 
 * Function Name : BQ24735_read
 * Description   : Read 16-bit value from BQ24735 register
 *                 BQ24735 returns: LSB first, then MSB
 * Arguments     : reg - Register address to read from
 * Return Value  : 16-bit register value, or 0xFFFF on error
 ******************************************************************************/
uint16_t BQ24735_read(uint8_t reg)
{
    uint8_t lsb, msb;

    EA = 0;   // Disable interrupts (good practice)

    I2C_start();
    if (I2STAT != 0x08) goto error;

    I2C_write(I2C_SLAVE_ADDRESS | I2C_W);
    if (I2STAT != 0x18) goto error;

    I2C_write(reg);
    if (I2STAT != 0x28) goto error;

    I2C_start();
    if (I2STAT != 0x10) goto error;   // Repeated START

    I2C_write(I2C_SLAVE_ADDRESS | I2C_R);
    if (I2STAT != 0x40) goto error;

    // BQ24735 sends LSB first, then MSB
    lsb = I2C_read(I2C_ACK);
    if (I2STAT != 0x50) goto error;

    msb = I2C_read(I2C_NACK);
    if (I2STAT != 0x58) goto error;

    I2C_stop();
    EA = 1;

    // Combine: MSB in upper byte, LSB in lower byte
    return ((uint16_t)msb << 8) | lsb;

error:
    I2C_stop();
    EA = 1;
    return 0xFFFF;   // Explicit error
}

/****************************************************************************** 
 * Function Name : BQ24735_read_retry
 * Description   : Read with retry mechanism for better reliability
 * Arguments     : reg - Register address to read from
 * Return Value  : 16-bit register value, or 0xFFFF after all retries fail
 ******************************************************************************/
uint16_t BQ24735_read_retry(uint8_t reg)
{
    uint8_t i;
    uint16_t val;

    for (i = 0; i < 3; i++)   // 3 retries
    {
        val = BQ24735_read(reg);
        if (val != 0xFFFF)
            return val;

        Timer2_Delay(24000000, 1, 10, 1000); // 10 ms backoff (increased from 2ms)
    }

    return 0xFFFF;
}