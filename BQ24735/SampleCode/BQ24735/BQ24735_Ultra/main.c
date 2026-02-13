/**
 ******************************************************************************
 *
 * @author     Bacancy Systems LLP 
 * @date       13, September, 2021
 * @File_name: main.c
 ******************************************************************************
 */

/************************************************************************
 * INCLUDES
 ************************************************************************/
#include "MS51_16K.H"
#include "i2c.h"
#include "math.h"
#include "uart.h"

#define VERSION_MAJOR                   (1U)
#define VERSION_MINOR                   (0U)
#define VERSION_PATCH                   (0U)
#define FW_VERSION                      ((VERSION_MAJOR * 100) + (VERSION_MINOR * 10) + (VERSION_PATCH))


/****************************************************************************
 * MACROS - BQ24735 REGISTER ADDRESSES
 ***************************************************************************/
#define INPUT_CURRENT       0x3F
#define CHARGER_VOLTAGE     0x15
#define CHARGER_CURRENT     0x14
#define CHARGE_OPTION       0x12
#define MANUFACTURER_ID     0xFE
#define DEVICE_ID           0xFF

/****************************************************************************
 * BQ24735 CHARGE CURRENT VALUES
 ****************************************************************************/
#define CHARGE_CURRENT_64MA     0x0040
#define CHARGE_CURRENT_128MA    0x0080
#define CHARGE_CURRENT_256MA    0x0100
#define CHARGE_CURRENT_512MA    0x0200
#define CHARGE_CURRENT_1024MA   0x0400

#define INPUT_CURRENT_1024MA    0x0400
#define INPUT_CURRENT_2048MA    0x0800

#define CHARGE_VOLTAGE_12_6V    0x31E0   // 12.6V (4.2V per cell)
#define CHARGE_VOLTAGE_13_3V    0x3340   // 13.3V (4.43V per cell)

/****************************************************************************
 * USER CONFIGURATION - CRITICAL: USE HIGHER CURRENT!
 ****************************************************************************/
#define SELECTED_CHARGE_CURRENT   CHARGE_CURRENT_512MA     // <-- MUST BE =512mA
#define SELECTED_INPUT_CURRENT    INPUT_CURRENT_1024MA
#define SELECTED_CHARGE_VOLTAGE   CHARGE_VOLTAGE_12_6V

/****************************************************************************
 * ERROR HANDLING
 ****************************************************************************/
#define MAX_CONFIG_RETRIES        6

/****************************************************************************
 * BQ24735 TIMING REQUIREMENTS
 ****************************************************************************/
#define BQ_POWERUP_DELAY_MS       100    // Wait for BQ24735 power-on reset
#define BQ_SOFT_RESET_DELAY_MS    200    // Delay after soft reset
#define BQ_CONFIG_DELAY_MS        50    // Delay between configuration steps
#define BQ_VERIFY_DELAY_MS        100    // Delay before verification reads

/****************************************************************************
 * AC MAINS DEBOUNCING
 ****************************************************************************/
#define AC_STABLE_CHECKS          4      // Must be stable for 3 checks
#define AC_DEBOUNCE_DELAY_MS      50    // Check every 100ms

/****************************************************************************
 * OTHER MACROS
 ****************************************************************************/
#define TIMER_DELAY_US      1000
#define SYS_CLOCK_FREQ      24000000UL
#define TIMER0_PRESCALER    12UL
#define TIMER0_TICKS_PER_US (SYS_CLOCK_FREQ / TIMER0_PRESCALER / 1000000UL)

#define MAINS_DETECT_PIN_SET_INPUT()    P03_INPUT_MODE
#define BATTERY_CHARGE_THRESHOLD        (10.5f)

#define SET_GUN_PINS_AS_OUTPUT()  do { \
    P12_PUSHPULL_MODE; \
    P11_PUSHPULL_MODE; \
    P10_PUSHPULL_MODE; \
    P00_PUSHPULL_MODE; \
} while(0)

#define GUN1_LOCK_PIN       P12
#define GUN1_UNLOCK_PIN     P11
#define GUN2_LOCK_PIN       P10
#define GUN2_UNLOCK_PIN     P00
#define IS_AC_MAINS_PRESENT (P03 == 0)

/****************************************************************************
 * GLOBAL VARIABLES
 ****************************************************************************/
bit      bBQ24735_OK = 0;
uint8_t  u8ConsecutiveErrors = 0;
uint8_t	 u8TH0_reload, u8TL0_reload;
uint16_t u16Cnt = 0, u16AdcVal = 0, u16FilterVal = 0;
uint32_t u32AccumulatedVal;
float    fBattVolt = 0;
float    fAdcVoltage;
float    fInputVoltage;
uint16_t u16ReadWhileACPresent = 0;
uint16_t u16StatusTimer = 0;
uint16_t u16Reset_BQ_IC = 0;
uint16_t u16VerifyCounter = 0;
/****************************************************************************
 * FUNCTION PROTOTYPES
 ****************************************************************************/
void vADC_Init(void);
void vUart_Init(void);
void vInit_BQ24735(void);
void vMeasureBattery(void);
void vUnlock_BothGuns(void);
void vRecover_I2C_Bus(void);
void vTimer0_Init(void);
void vBQ24735_HardReset(void);
bit bBQ24735_WaitReady(void);
bit bConfigure_BQ24735(void);
bit bCheck_AC_Stable(bit bExpectedState);
float fConvertADCtoInputVoltage(uint16_t u16AdcCount);
uint16_t u16ADC_GetData(void);

/****************************************************************************
 * TIMER0 ISR
 ****************************************************************************/
/****************************************************************************
 * TIMER0 ISR — 1ms tick
 ****************************************************************************/
void Timer0_ISR(void) interrupt 1
{
    TH0 = u8TH0_reload;
    TL0 = u8TL0_reload;
    TF0 = 0;

    // Increment all timers
   
    u16StatusTimer++;
		u16Reset_BQ_IC++;
   

    // Trigger ADC every tick
    clr_ADCF;
    set_ADCS;
}

/******************************************************************************
 * MAIN FUNCTION
 ******************************************************************************/
void main (void) 
{
    uint8_t cnt = 100;
   
    bit bIsAcMainsPrevState = 1;
	
    SET_GUN_PINS_AS_OUTPUT();
    MAINS_DETECT_PIN_SET_INPUT();
    vUart_Init();
    vADC_Init();
    vTimer0_Init();
    printf("\r\n========================================\r\n");
		printf("BQ24735 Charger Controller FW Version: %u.%u.%u\r\n",  VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);

    printf("  Charge Current: %dmA\r\n", SELECTED_CHARGE_CURRENT);
    printf("========================================\r\n");

    GUN1_LOCK_PIN = 0;
    GUN1_UNLOCK_PIN = 0;
    GUN2_LOCK_PIN = 0;
    GUN2_UNLOCK_PIN = 0;
	
    clr_ADCF;
    set_ADCS;
    
    /******************************************************************************
     * CRITICAL: Wait for BQ24735 Power-On Reset
     ******************************************************************************/
    printf("\r\nWaiting for BQ24735 power-on reset...\r\n");
    Timer2_Delay(24000000, 1, BQ_POWERUP_DELAY_MS, 1000);
    
    Init_I2C();
    Timer2_Delay(24000000, 1, BQ_POWERUP_DELAY_MS, 1000);
    
    /******************************************************************************
     * INITIAL BQ24735 CONFIGURATION
     ******************************************************************************/
    printf("\r\nConfiguring BQ24735...\r\n");
    
		vInit_BQ24735();
   
    // Stabilize ADC readings
    while(cnt)
    {
        vMeasureBattery();
        cnt--;
        Timer2_Delay(24000000, 1, 20, 1000);
    }
    
	/******************************************************************************
	 * MAIN LOOP - MONITOR CHARGING WITH PROPER AC DEBOUNCING
	 ******************************************************************************/
	while (1)
	{
		/******************************************************************************
		 * AC MAINS PRESENT - CHARGING MODE
		 ******************************************************************************/
			if (IS_AC_MAINS_PRESENT)
			{
					/* -------- AC JUST BECAME PRESENT -------- */
					if (bIsAcMainsPrevState == 0)
					{							
						 // Verify AC is stable before proceeding
							if(!bCheck_AC_Stable(1))
							{
									printf("AC unstable - ignoring transient\r\n");
									continue;
							}
							else{
									printf("\r\n========================================\r\n");
									printf("AC POWER RESTORED (stable)\r\n");
									printf("========================================\r\n");
									vInit_BQ24735();

									// Reset state
									u16VerifyCounter = 0;
									bIsAcMainsPrevState = 1;
						}	
				}
			}
			/******************************************************************************
			 * AC MAINS LOST - SAFETY MODE
			 ******************************************************************************/
			else
			{
					if (bIsAcMainsPrevState == 1)
					{
							// Verify AC is actually lost (debounce)
							if(!bCheck_AC_Stable(0))
							{
									printf("AC flicker detected - stabilizing...\r\n");
								  continue;  
							}
							else
							{
								printf("\r\n========================================\r\n");
								printf("!!! AC POWER LOST (confirmed) !!!\r\n");
								printf("========================================\r\n");

								// Unlock guns
								printf("Unlocking guns...\r\n");
								vUnlock_BothGuns();
								
								bIsAcMainsPrevState = 0;
								u16VerifyCounter = 0;
								
								printf("Waiting for AC power...\r\n");
								printf("========================================\r\n\r\n");
							}
					}
			}
	
		/*print batter status here*/
		if(u16StatusTimer>1000)
		{
				vMeasureBattery();
				/* -------- STATUS PRINT -------- */
				printf("BATT=%.2fV  (runtime=%u sec)\r\n", fBattVolt, u16VerifyCounter++);			
				u16StatusTimer = 0;
		}
		if(u16Reset_BQ_IC>60000)
		{
			printf("\r\n========================================\r\n");
			printf("Periodic BQ RESET... \r\n");
			printf("========================================\r\n");
			vInit_BQ24735();
			u16Reset_BQ_IC = 0;
		}
	
	}
}

/******************************************************************************
 * Function: bCheck_AC_Stable
 * Description: Verify AC mains state is stable (debouncing)
 * Arguments: bExpectedState - Expected AC state (1=present, 0=absent)
 * Returns: 1 if stable, 0 if unstable
 ******************************************************************************/
bit bCheck_AC_Stable(bit bExpectedState)
{
    uint8_t u8StableCount = 0;
    uint8_t i;
    
    for(i = 0; i < AC_STABLE_CHECKS; i++)
    {
        if(IS_AC_MAINS_PRESENT == bExpectedState)
        {
            u8StableCount++;
        }
        else
        {
            return 0;  // Unstable - state changed
        }
        Timer2_Delay(24000000, 1, AC_DEBOUNCE_DELAY_MS, 1000);
    }
    
    return (u8StableCount == AC_STABLE_CHECKS);
}

/******************************************************************************
 * Function: bBQ24735_WaitReady
 ******************************************************************************/
bit bBQ24735_WaitReady(void)
{
    uint8_t u8Attempt;
    uint16_t u16ManufID;
    
    for(u8Attempt = 0; u8Attempt < 10; u8Attempt++)
    {
        u16ManufID = BQ24735_read_retry(MANUFACTURER_ID);
        
        if(u16ManufID != 0xFFFF && u16ManufID != 0x0000)
        {
            printf("READY (ID=0x%04X) - ", u16ManufID);
            return 1;
        }
        
        Timer2_Delay(24000000, 1, 100, 1000);
    }
    
    printf("TIMEOUT ");
    return 0;
}

/******************************************************************************
 * Function: vBQ24735_HardReset
 ******************************************************************************/
void vBQ24735_HardReset(void)
{
    printf("  Resetting BQ24735...\r\n");
    
    BQ24735_write(CHARGER_CURRENT, 0x0000);
    Timer2_Delay(24000000, 1, 50, 1000);
    
    vRecover_I2C_Bus();
    
    Timer2_Delay(24000000, 1, BQ_SOFT_RESET_DELAY_MS, 1000);
}

/******************************************************************************
 * Function: bConfigure_BQ24735
 ******************************************************************************/
/*
Here's the clean table:

| Bit | Value | Field | Description |
|-----|-------|-------|-------------|
| 15 | 1 | CHARGE_INHIBIT | Charger disabled — inhibits charging |
| 14 | 0 | ACOC | AC overcurrent threshold = 133% of nominal |
| 13 | 0 | BOOST_MODE | Boost mode disabled |
| 12 | 1 | Reserved | Factory default |
| 11 | 0 | Reserved | — |
| 10 | 0 | Reserved | — |
| 9 | 0 | Reserved | — |
| 8 | 0 | Reserved | — |
| 7 | 0 | LOWPOWER | Normal power mode |
| 6 | 0 | Reserved | — |
| 5 | 1 | EMI_FREQ_ADJ | Switching frequency adjusted for EMI |
| 4 | 0 | Reserved | — |
| 3 | 0 | Reserved | — |
| 2 | 0 | IOUT | IOUT pin reports input current |
| 1 | 0 | LEARN | Battery learn cycle disabled |
| 0 | 0 | IFAULT | No fault |
*/
bit bConfigure_BQ24735(void)
{
    uint16_t u16Read;
    uint16_t u16ChargeOptions;

    // Verify communication
    u16Read = BQ24735_read_retry(MANUFACTURER_ID);
    if(u16Read == 0xFFFF || u16Read == 0x0000)
    {
        printf("COMM_FAIL ");
        return 0;
    }

    // Disable charging first
    BQ24735_write(CHARGER_CURRENT, 0x0000);
    Timer2_Delay(24000000, 1, BQ_CONFIG_DELAY_MS, 1000);

    // Configure options
    u16ChargeOptions = 0x1020;
    BQ24735_write(CHARGE_OPTION, u16ChargeOptions);
    Timer2_Delay(24000000, 1, BQ_CONFIG_DELAY_MS, 1000);

    // Set input current
    BQ24735_write(INPUT_CURRENT, SELECTED_INPUT_CURRENT);
    Timer2_Delay(24000000, 1, BQ_CONFIG_DELAY_MS, 1000);

    // Set voltage
    BQ24735_write(CHARGER_VOLTAGE, SELECTED_CHARGE_VOLTAGE);
    Timer2_Delay(24000000, 1, BQ_CONFIG_DELAY_MS, 1000);

    // Enable charging
    BQ24735_write(CHARGER_CURRENT, SELECTED_CHARGE_CURRENT);
    Timer2_Delay(24000000, 1, BQ_VERIFY_DELAY_MS, 1000);

    // Verify (attempt 1)
    u16Read = BQ24735_read_retry(CHARGER_CURRENT);
    if(u16Read == 0xFFFF || u16Read == 0x0000)
    {
        printf("VERIFY_FAIL ");
        return 0;
    }
    
    // Verify (attempt 2)
    Timer2_Delay(24000000, 1, 200, 1000);
    u16Read = BQ24735_read_retry(CHARGER_CURRENT);
    if(u16Read != SELECTED_CHARGE_CURRENT)
    {
        printf("VERIFY2_FAIL (got 0x%04X) ", u16Read);
        return 0;
    }

    printf("CFG_OK\r\n");
    return 1;
}

/******************************************************************************
 * Function: vRecover_I2C_Bus
 ******************************************************************************/
void vRecover_I2C_Bus(void)
{
    clr_I2CON_I2CEN;
    Timer2_Delay(24000000, 1, 100, 1000);
    Init_I2C();
    Timer2_Delay(24000000, 1, 100, 1000);
}

/******************************************************************************
 * Function: vUnlock_BothGuns
 ******************************************************************************/
void vUnlock_BothGuns(void)
{
    GUN1_LOCK_PIN = 0;
    GUN2_LOCK_PIN = 0;
	 
    GUN1_UNLOCK_PIN = 1;
    Timer2_Delay(24000000, 1, 100, 1000);
    GUN1_UNLOCK_PIN = 0;
    Timer2_Delay(24000000, 1, 2000, 1000);

    GUN2_UNLOCK_PIN = 1;
    Timer2_Delay(24000000, 1, 100, 1000);
    GUN2_UNLOCK_PIN = 0;
    Timer2_Delay(24000000, 1, 1000, 1000);
}

/******************************************************************************
 * Function: vMeasureBattery
 ******************************************************************************/
void vMeasureBattery(void)
{
    if (ADCF)
    {
        u16AdcVal = u16ADC_GetData();
        u32AccumulatedVal -= (u32AccumulatedVal >> 4);
        u32AccumulatedVal += u16AdcVal;
        u16FilterVal = u32AccumulatedVal >> 4;
        fBattVolt = fConvertADCtoInputVoltage(u16FilterVal);
        clr_ADCF;
        set_ADCS;
    }
}

/******************************************************************************
 * Function: fConvertADCtoInputVoltage
 ******************************************************************************/
float fConvertADCtoInputVoltage(uint16_t u16AdcCount) 
{
    const float fR1 = 147.0f;
    const float fR2 = 51.0f;
    const float fDIVIDER_RATIO = fR2 / (fR1 + fR2);
    const float fADC_REF_VOLTAGE = 3.3f;
    const uint16_t u16ADC_MAX_COUNT = 4095;
		
    fAdcVoltage = ((float)u16AdcCount * fADC_REF_VOLTAGE) / u16ADC_MAX_COUNT;
    fInputVoltage = fAdcVoltage / fDIVIDER_RATIO;
    return fInputVoltage;
}

/******************************************************************************
 * Function: vADC_Init
 ******************************************************************************/
void vADC_Init()
{
    ADCCON1 &= ~(0x30);
    ADCCON1 |= (0x30);
    P04_INPUT_MODE;
    AINDIDS |= (1 << 5);
    ADCCON1 |= (1 << 0);
    ADCCON0 &= ~(0x0F);
    ADCCON0 |= (5 & 0x07);
}

/******************************************************************************
 * Function: u16ADC_GetData
 ******************************************************************************/
uint16_t u16ADC_GetData(void)
{
    return ((ADCRH << 4) | (ADCRL & 0x0F));
}

/******************************************************************************
 * Function: vUart_Init
 ******************************************************************************/
void vUart_Init(void)
{
    MODIFY_HIRC(HIRC_24);
    P06_QUASI_MODE;
    UART_Open(24000000, UART0_Timer3, 115200);
    ENABLE_UART0_PRINTF;
}


void vInit_BQ24735()
{
	 uint8_t u8ConfigAttempt;
	 bBQ24735_OK = 0;
  	for(u8ConfigAttempt = 0; u8ConfigAttempt < MAX_CONFIG_RETRIES; u8ConfigAttempt++)
    {
        printf("Attempt %d/%d... ", u8ConfigAttempt + 1, MAX_CONFIG_RETRIES);
        
        if(!bBQ24735_WaitReady())
        {
            printf("NOT READY - ");
        }
        
        if(bConfigure_BQ24735())
        {
            printf("SUCCESS!\r\n");
            bBQ24735_OK = 1;
            break;
        }
        else
        {
            printf("FAILED\r\n");
            if(u8ConfigAttempt < MAX_CONFIG_RETRIES - 1)
            {
                printf("  Performing hard reset...\r\n");
                vBQ24735_HardReset();
                Timer2_Delay(24000000, 1, 500, 1000);
            }
        }
    }
    
    if(!bBQ24735_OK)
    {
        printf("\r\n!!! BQ24735 INIT FAILED !!!\r\n");
        printf("System will monitor only (no charging)\r\n");
    }
    else
    {
        printf("\r\nBQ24735 Ready. Entering main loop...\r\n");
        printf("========================================\r\n\r\n");
    }
    
    u8ConsecutiveErrors = 0;
}

/******************************************************************************
 * Function: vTimer0_Init
 * Description: Continuous 1ms tick for u16Cnt and ADC triggering
 ******************************************************************************/
void vTimer0_Init(void)
{
    // 16-bit mode, 1ms interval
    // Reload = 65536 - (24000000 / 12 / 1000) = 65536 - 2000 = 63536
    uint16_t u16Reload = 65536 - 
                        (SYS_CLOCK_FREQ / TIMER0_PRESCALER / 1000UL);

    u8TH0_reload = (uint8_t)(u16Reload >> 8);
    u8TL0_reload = (uint8_t)(u16Reload & 0xFF);

    TH0 = u8TH0_reload;
    TL0 = u8TL0_reload;

    TMOD &= 0xF0;
    TMOD |= 0x01;       // Mode 1 - 16 bit continuous
    
    TF0 = 0;
    ET0 = 1;
    EA  = 1;
    set_TR0;            // Start and never stop
}