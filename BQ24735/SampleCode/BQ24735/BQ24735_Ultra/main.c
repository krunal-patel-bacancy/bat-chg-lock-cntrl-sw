/**
 ******************************************************************************
 *
 * @author     Bacancy Systems LLP 
 * @date       13, September, 2021
 * @File_name: main.c
 ******************************************************************************
 */
//  File Function: MS51FB9AE SOC demo code

/* project pin details 
+--------+-----------------------+
| GPIO   | Description           |
+--------+-----------------------+
| P1.4   | SDA [I2C]             |
| P1.3   | SCL [I2C]             |
+--------+-----------------------+
| P0.3   | mains detect [Input]] | 
+--------+-----------------------+
| P1.2   | Gun1_lock    [output] |
| P1.1   | Gun1_unlock  [output] |
+--------+-----------------------+
| P1.0   | Gun2_lock    [output] |
| P0.0   | Gun2_unlock  [output] |
+--------+-----------------------+
| P0.4   |
|[ADCch5]| BatterySense [ADC]    |
+--------+----------------------*/

/************************************************************************
 * INCLUDES
 ************************************************************************/
#include "MS51_16K.H"
#include "i2c.h"
#include "math.h"
#include "uart.h"


/****************************************************************************
 * MACROS
 ***************************************************************************/
#define INPUT_CURRENT    0x3F
#define CHARGER_VOLTAGE  0x15
#define CHARGER_CURRENT  0x14
#define CHECK_BIT(x, pos) ((x) & (1UL << pos) )

#undef UART0_DEBUG
//  --------------------------------- TIMER INTERVAL CALCULATION -------------------------------------------------------------
#define TIMER_DELAY_US      1000    // Desired delay in microseconds (e.g., 1000 for 1ms)
#define SYS_CLOCK_FREQ      24000000UL  // System Clock = 24 MHz
#define TIMER0_PRESCALER    12UL        // Fsys/12 for Timer0
#define TIMER0_TICKS_PER_US (SYS_CLOCK_FREQ / TIMER0_PRESCALER / 1000000UL)  // = 2 ticks per �s

//  --------------------------------- PROJECT PINS -------------------------------------------------------------
#define MAINS_DETECT_PIN_SET_INPUT() 		P03_INPUT_MODE

#define SET_GUN_PINS_AS_OUTPUT()  			do { \
		/* GUN1 LOCK */   P12_PUSHPULL_MODE; \
		/* GUN1 UNLOCK */ P11_PUSHPULL_MODE; \
		/* GUN2 LOCK */   P10_PUSHPULL_MODE; \
		/* GUN2 UNLOCK */ P00_PUSHPULL_MODE; \
} while(0)

#define SET_GUN_PINS_AS_INPUT()  do { \
   /* GUN1 LOCK   */ P12_INPUT_MODE; \
   /* GUN1 UNLOCK */ P11_INPUT_MODE; \
   /* GUN2 LOCK   */ P10_INPUT_MODE; \
   /* GUN2 UNLOCK */ P00_INPUT_MODE; \
} while(0)


#define GUN1_LOCK_PIN       						P12
#define GUN1_UNLOCK_PIN     						P11
#define GUN2_LOCK_PIN       						P10
#define GUN2_UNLOCK_PIN     						P00
#define IS_AC_MAINS_PRESENT   				 (P03 == 0)  // returns true if mains signal is active

/****************************************************************************
 * GLOBAL VARIABLES
 ****************************************************************************/
bit 		 bGunPinsConfigured = 1;		

uint8_t	 u8TH0_reload,  // Global variables to hold calculated TH0 and TL0
				 u8TL0_reload;

uint16_t u16Cnt = 0,
				 u16AdcVal = 0,
				 u16FilterVal = 0;

uint32_t u32AccumulatedVal;

float    fBattVolt = 0;		


/****************************************************************************
 * PRIVATE FUNCTIONS
 ****************************************************************************/
void vADC_Init(void);
void vUart_Init(void);
void vMeasureBattery(void);
void vGunInputAndI2CCmd(void);
void vUnlock_BothGuns(void);
void vTimer0_Init_Dynamic(uint16_t u16Delay_us);
void vI2C_Send_charging_commands(void);
float fConvertADCtoInputVoltage(uint16_t u16AdcCount);
uint16_t u16ADC_GetData(void);

/****************************************************************************
 * PHERIPHERAL ISR's
 ****************************************************************************/

// Timer0 ISR (executes every 1ms or as per TIMER_DELAY_US)
void Timer0_ISR(void) interrupt 1
{
    _push_(SFRS);

    TH0 = u8TH0_reload;
    TL0 = u8TL0_reload;
    TF0 = 0;

	if(++u16Cnt>250){
		u16Cnt = 0;
	}
	 // Start ADC conversion
    clr_ADCF;
    set_ADCS;
	
    _pop_(SFRS);
}

/******************************************************************************
The main C function.  Program execution starts
here after stack initialization.
******************************************************************************/
void main (void) 
{
  bit bIsAcMainsPrevState = 1; 			// Track previous AC mains state; assume AC is present at startup
	
	SET_GUN_PINS_AS_OUTPUT();					//set pins initially as input for safe operation
	MAINS_DETECT_PIN_SET_INPUT(); 		// Initialize AC mains detection pin
	
	vUart_Init(); 										// Initialize UART0 for printf debugging
	vADC_Init();					 						// Initialize ADC for battery voltage measurement
	vTimer0_Init_Dynamic(TIMER_DELAY_US);  // Initialize Timer0 with dynamic delay [currentl 1ms is used]
	
	Init_I2C();														 // Initialize I2C communication
	
	vI2C_Send_charging_commands();

	//Initially Sets all pins to LOW
	GUN1_LOCK_PIN    = 0; 		
	GUN1_UNLOCK_PIN  = 0; 
	GUN2_LOCK_PIN    = 0;
	GUN2_UNLOCK_PIN  = 0;
	
/******************************************************************************
 * Main application loop
 * - Monitors AC mains status
 * - Unlocks both guns when AC is lost (pulse once)
 * - Continuously measures battery voltage
 ******************************************************************************/
while (1)
{
    if (IS_AC_MAINS_PRESENT)
    {
				vGunInputAndI2CCmd();
				bIsAcMainsPrevState = 1;     // AC mains is confirmed present
    }
    else
    {
        if (bIsAcMainsPrevState == 1)
        {
						Timer2_Delay(24000000,1,1000,1000);		 // Wait 1 second to confirm AC loss

            if (!IS_AC_MAINS_PRESENT) 					  // Confirm AC is still not present
            {
                vUnlock_BothGuns();     					// Apply 100 ms pulse to both unlock pins
                bIsAcMainsPrevState = 0; 					// Mark AC as gone to prevent retrigger
            }
        }
    }
		vMeasureBattery();
}
}

/****************************************************************************** 
 * Function Name : vUnlock_BothGuns
 * Description   : Releases control of both Gun1 and Gun2 solenoids by unlocking them.
 * Arguments     : None
 * Return Value  : None
 ******************************************************************************/
void vUnlock_BothGuns(void)
{
		GUN1_LOCK_PIN    = 0; // we dont have control guns so set both lock pins to low
		GUN2_LOCK_PIN    = 0;
	 
		GUN1_UNLOCK_PIN  = 1;  // Apply pulse to Gun1 & Gun2 unlock pin
		Timer2_Delay(24000000,1,100,1000); // to unlock the Guns give a pulse duration of 100ms only
		GUN1_UNLOCK_PIN  = 0;  // Clear pulse

		Timer2_Delay(24000000,1,2000,1000); //short delay for safe operation

		GUN2_UNLOCK_PIN  = 1;  
		Timer2_Delay(24000000,1,100,1000); // to unlock the Guns give a pulse duration of 100ms only
		GUN2_UNLOCK_PIN  = 0;  
	
		Timer2_Delay(24000000,1,1000,1000); //short delay for safe operation
	
		bGunPinsConfigured = 1;  //Again whenever AC mains present then we have to reinit pins as Input so set it here
}
/****************************************************************************** 
 * Function Name : vGUN_PIN_SET_Input_mode
 * Description   : In this mode device not taking any control.
 * Arguments     : None
 * Return Value  : None
 ******************************************************************************/
void vGunInputAndI2CCmd()
{
	if(bGunPinsConfigured)
	{
		vI2C_Send_charging_commands();

		//for one time execution reset the flag
		bGunPinsConfigured = 0;
	}	
}

/****************************************************************************** 
 * Function Name : vI2C_Send_charging_commands
 * Description   : This function sends charging command to BQ IC [ for battery charge require voltage and current configuration].
 * Arguments     : None
 * Return Value  : None
 ******************************************************************************/
void vI2C_Send_charging_commands()
{
	unsigned char manf_id ,dev_id;		// For storing manufacturer and device IDs 
	int chargOption;									// Charger option placeholder (currently unused)
	int bdta;													// Battery data placeholder (currently unused)

	manf_id = BQ24735_read(0xFE);					 // Read manufacturer ID
	dev_id = BQ24735_read(0xFF);					 // Read device ID

  Timer2_Delay(24000000,1,1000,1000);		 // Short delay before configuring charger
  
	//BQ24735_write(0x12, 0xFF02);    // Set input current limit to 1024 mA	
	
	BQ24735_write(INPUT_CURRENT,0x400);    // Set input current limit to 1024 mA
	
	/*  
	/---------------- Charge current configuration values ---------------------/
	-----------------------------------
	Register 			Charge     
	value					current										voltage
								value	
	-------------------------------------
	0x0040				64ma [Old  , New1,  ] 		2.1
	0x0080				128ma[80ma,  28ma,	]	 		11.5
	0x0100	 			256ma[220ma, 42ma,  ]  		8.5
	0x0200	 			512mA[480ma, 84ma,  ] 		10.5      
	0x0400	 			1 amp[770ma, 700ma, ]  		10.65       
	-------------------------------------	
#define INPUT_CURRENT    0x3F
#define CHARGER_VOLTAGE  0x15
#define CHARGER_CURRENT  0x14
	
	*/
	BQ24735_write(CHARGER_CURRENT,0x0400); // Set charging current to 1024 mA
	BQ24735_write(CHARGER_VOLTAGE,0x3400); // Set charging voltage to 13312 mV

  Timer2_Delay(24000000,1,5000,1000);		 // Short delay before configuring charger
}

/******************************************************************************
 * Function Name: vMeasureBattery
 * Description  : measures the battery voltage
 * Arguments    : none
 * Return Value : None
 ******************************************************************************/
void vMeasureBattery(void)
{
    // Check if ADC conversion is done
    if (ADCF)
    {
			u16AdcVal = u16ADC_GetData();
			u32AccumulatedVal -= (u32AccumulatedVal >>4);
			u32AccumulatedVal += u16AdcVal;
			u16FilterVal = 	u32AccumulatedVal >>4;			
			fBattVolt = fConvertADCtoInputVoltage(u16FilterVal);
			clr_ADCF;
    }

	 //to print adc values enable this it will print it UART_0	
	 //#ifdef UART0_DEBUG    
			//printf("\n ADC: %d, BattVol: %f", u16FilterVal,fBattVolt);
			//Timer2_Delay(24000000,1,100,1000);
	 //#endif   
}

/******************************************************************************
 * Function Name: fConvertADCtoInputVoltage
 * Description  : Converts 12-bit ADC raw count to actual input voltage (before
 *                voltage divider). voltage divider R1 = 147k, R2 = 51k.
 * Arguments    : u16AdcCount - Raw ADC value (0 to 4095 for 12-bit ADC)
 * Return Value : float - Actual input voltage in volts
 ******************************************************************************/
float fConvertADCtoInputVoltage(uint16_t u16AdcCount)
{
    float fAdcVoltage;     // Voltage at ADC pin (after divider)
    float fInputVoltage;   // Actual input voltage (before divider)
 
		const float  fR1 = 147.0f;
		const float  fR2 = 51.0f;
    const float  fDIVIDER_RATIO   = fR2 / (fR1 + fR2);  // R2 / (R1 + R2)
    const float  fADC_REF_VOLTAGE = 3.3f;        // ADC reference voltage
		const uint16_t u16ADC_MAX_COUNT = 4095;       // 12-bit ADC maximum count

    // Step 1: Convert raw ADC count to voltage at the ADC pin
    fAdcVoltage = ((float)u16AdcCount * fADC_REF_VOLTAGE) / u16ADC_MAX_COUNT;

    // Step 2: Calculate the original input voltage before the voltage divider
    fInputVoltage = fAdcVoltage / fDIVIDER_RATIO;

    return fInputVoltage;
}

/******************************************************************************
 * Function Name: vADC_Init
 * Description  : This function initializes the ADC with software trigger mode.
 *                ADC is triggered using Timer0 at intervals defined by TIMER_DELAY_US.
 *                It configures the selected channel for analog input.
 * Arguments    : None
 * Return Value : None
 ******************************************************************************/
void vADC_Init()
{
    ADCCON1 &= ~(0x30);               // Clear ADC clock divider bits
    ADCCON1 |= (0x30);                // Set ADC clock divider

		P04_INPUT_MODE;										// adc channel 5 is used for batt sense which is on PO4 pin so it as first input

    AINDIDS |= (1 << 5);    // Enable analog input for the selected channel

    ADCCON1 |= (1 << 0);              // Enable the ADC module

    ADCCON0 &= ~(0x0F);               // Clear previous channel selection
    ADCCON0 |= (5 & 0x07);  // Select desired ADC channel
}

/******************************************************************************
 * Function Name: u16ADC_GetData
 * Description  : This function gives the adc converted result value
 * Arguments    : None
 * Return Value : 16bit adc value
 ******************************************************************************/
uint16_t u16ADC_GetData(void)
{
    return ((ADCRH << 4) | (ADCRL & 0x0F)); // return adc values 
}

/******************************************************************************
 * Function Name: vTimer0_Init_Dynamic
 * Description  : This function initializes Timer0 in 13-bit mode to generate
 *                a dynamic delay based on the input time in microseconds.
 *                It calculates the reload value and configures Timer0 to
 *                overflow after the specified delay.
 * Arguments    : u16Delay_us - Desired delay in microseconds (µs)
 * Return Value : None
 ******************************************************************************/
void vTimer0_Init_Dynamic(uint16_t u16Delay_us)
{
    uint16_t u16Reload;
    uint16_t u16Ticks = u16Delay_us * TIMER0_TICKS_PER_US;

    // Limit the maximum tick count to 13-bit timer max value (8192)
    if (u16Ticks >= 8192)
        u16Ticks = 8191;

    // Calculate the reload value to count from (8192 - desired ticks)
    u16Reload = 8192 - u16Ticks;

    // Extract high and low parts of the 13-bit reload value
    u8TH0_reload = (u16Reload >> 5) & 0xFF;  // Upper 8 bits
    u8TL0_reload = u16Reload & 0x1F;         // Lower 5 bits [ so total 13bits ]

    // Load values into Timer0 registers
    TH0 = u8TH0_reload;
    TL0 = u8TL0_reload;

    // Configure and start Timer0
    ENABLE_TIMER1_MODE0;        // Set Timer0 to 13-bit mode (Mode 0)
    TIMER0_FSYS_DIV12;          // Set clock source as system clock divided by 12
    ENABLE_TIMER0_INTERRUPT;    // Enable Timer0 interrupt (optional)
    ENABLE_GLOBAL_INTERRUPT;    // Enable global interrupts
    set_TCON_TR0;               // Start Timer0
}

/******************************************************************************
 * Function Name: vUart_Init
 * Description  : This function initializes UART0 for debugging purposes using
 *                printf. It sets the system clock, configures the UART pin,
 *                and sets the baud rate.
 * Arguments    : None
 * Return Value : None
 ******************************************************************************/
void vUart_Init(void)
{
    /* UART0 setting for printf function */
    MODIFY_HIRC(HIRC_24);                     // Set internal RC to 24 MHz
    P06_QUASI_MODE;                           // Set P0.6 as quasi mode for TX
    UART_Open(24000000, UART0_Timer3, 115200); // Initialize UART0 using Timer3, 115200 baud
    ENABLE_UART0_PRINTF;                      // Enable printf support via UART0
}
