
// This program converts the analog input from channel 5.
// Channel 5 is configured as single-ended input from PA05 (pin 6).
// External reference A is used from PA03 (pin 4), that should be connected to 3.3V. 

#include "samd20.h"
#include "nvm_data.h"

#define PE 4  // Prescaler as configured in REC_TC3_CTRLA
#define RATE 100000 // in Hz.  We want to interrupt every 10us
#define TC3_RELOAD (((F_CPU/PE)/RATE)-1)

#if (TC3_RELOAD > 255)
#error TC3_RELOAD is greater than 255
#endif

volatile int ISR_pwm1=150, ISR_pwm2=150, ISR_cnt=0;

unsigned char* ARRAY_PORT_PINCFG0 = (unsigned char*)&REG_PORT_PINCFG0;
unsigned char* ARRAY_PORT_PMUX0 = (unsigned char*)&REG_PORT_PMUX0;

int detect_coin()
int get_perimeter_reading()
void init_Clock48();
void UART3_init(uint32_t baud);
void printString (char * s);
void printNum(uint32_t v, int base, int digits);
void turn_around();
void coin_pickup_reverse();
void coin_pickup();
void servo(int n, int m);
void move_foward();


void Configure_TC2_servo ()
{
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TC2_TC3;
    REG_PM_APBCMASK |= PM_APBCMASK_TC2; // Enable TC2 bus clock
    
    REG_TC2_CTRLA = 1;              /* Software Reset */
    while (REG_TC2_CTRLA & 1) {}    /* Wait till out of reset */
    REG_TC2_CTRLA = 0x0560;         /* Prescaler: GCLK_TC/64, 16-bit mode, MPWM.  Check page 490*/
    REG_TC2_COUNT16_CC0 = (F_CPU/(64*50))-1; /* Match value 0 */
    REG_TC2_CTRLA |= 2;             /* Enable */

	REG_TC2_COUNT16_CC1 = (F_CPU/(64*50*20))-1;

    /* Enable & configure alternate function F for pin PA17 (pin 18) Page 22*/
    PORT->Group[0].PINCFG[17].bit.PMUXEN = 1;
    PORT->Group[0].PMUX[8].reg = 0x50;
}

void Configure_TC3_servo ()
{
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TC2_TC3;
    REG_PM_APBCMASK |= PM_APBCMASK_TC3; // Enable TC3 bus clock
    
    REG_TC3_CTRLA = 1;              /* Software Reset */
    while (REG_TC3_CTRLA & 1) {}    /* Wait till out of reset */
    REG_TC3_CTRLA = 0x0560;         /* Prescaler: GCLK_TC/64, 16-bit mode, MPWM.  Check page 490*/
    REG_TC3_COUNT16_CC0 = (F_CPU/(64*50))-1; /* Match value 0 */
    REG_TC3_CTRLA |= 2;             /* Enable */

	REG_TC3_COUNT16_CC1 = (F_CPU/(64*50*20))-1;

    /* Enable & configure alternate function E for pin PA15 (pin 16) Page 22*/
    PORT->Group[0].PINCFG[15].bit.PMUXEN = 1;
    PORT->Group[0].PMUX[7].reg = 0x40;
}

void Configure_TC3 (void)
{
    __disable_irq();
    // Configure Clocks
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TC2_TC3;
    REG_PM_APBCMASK |= PM_APBCMASK_TC3; // Enable TC3 bus clock
    REG_TC3_CTRLA = 1;              // reset TC3 before configuration
    while (REG_TC3_CTRLA & 1);      // wait till out of reset
    REG_TC3_CTRLA = 0x0204;         // prescaler /64, 8-bit mode, NFRQ. Check page 681 of datasheet
    REG_TC3_COUNT8_PER=TC3_RELOAD;  // TOP count value in 8-bit mode
    REG_TC3_CTRLA |= 2;             // enable TC3
    REG_TC3_INTENSET = 1;           // enable overflow interrupt
    NVIC_EnableIRQ(TC3_IRQn);       // enable TC3 interrupt in NVIC
    __enable_irq();                 // enable interrupt globally

}

void TC3_Handler(void)
{
    REG_TC3_INTFLAG = 1; // clear OVF flag
    
	ISR_cnt++;
	if(ISR_cnt==ISR_pwm1)
	{
		REG_PORT_OUTCLR0 = PORT_PA08;
	}
	if(ISR_cnt==ISR_pwm2)
	{
		REG_PORT_OUTCLR0 = PORT_PA09;
	}
	if(ISR_cnt>=2000)
	{
		ISR_cnt=0; // 2000 * 10us=20ms
		REG_PORT_OUTSET0 = PORT_PA08;
		REG_PORT_OUTSET0 = PORT_PA09;
	}	
}

// This function read the period of the signal connecte to port PA14 (pin 15 of QFP32)
uint32_t GetPeriod (int n)
{
    int i;
    // Configure SysTick
    SysTick->LOAD = 0xffffff; // Reload with max number of clocks (SysTick is 24-bit)
    SysTick->VAL = 0;         // clear current value register
    SysTick->CTRL = 0x5;      // Enable the timer

    while ((REG_PORT_IN0 & PORT_PA15)!=0) // Wait for zero
    {
    	if (SysTick->CTRL & 0x10000) return 0;
    }
    while ((REG_PORT_IN0 & PORT_PA15)==0) // Wait for one
    {
    	if (SysTick->CTRL & 0x10000) return 0;
    }
    SysTick->CTRL = 0; // Stop the timer (Enable = 0)


    // Configure SysTick again
    SysTick->LOAD = 0xffffff;  // Reload with max number of clocks (SysTick is 24-bit)
    SysTick->VAL = 0;          // clear current value register
    SysTick->CTRL = 0x5;       // Enable the timer

    for(i = 0; i < n; i++)
    {
	    while ((REG_PORT_IN0 & PORT_PA15)!=0)
	    {
	    	if (SysTick->CTRL & 0x10000) return 0;
	    }
	    while ((REG_PORT_IN0 & PORT_PA15)==0)
	    {
	    	if (SysTick->CTRL & 0x10000) return 0;
	    }
    }
    SysTick->CTRL = 0; // Stop the timer (Enable = 0)
    
    return (0xffffff-SysTick->VAL);
}

void delayMs(int n)
{
    int i;
    // Configure SysTick
    SysTick->LOAD = (F_CPU/1000L) - 1; // Reload with number of clocks per millisecond
    SysTick->VAL = 0;         // clear current value register
    SysTick->CTRL = 0x5;      // Enable the timer

    for(i = 0; i < n; i++)
    {
        while((SysTick->CTRL & 0x10000) == 0); // wait until the COUNTFLAG is set
    }
    SysTick->CTRL = 0; // Stop the timer (Enable = 0)
}

void ADC_init (void)
{
	PM->APBCMASK.reg |= PM_APBCMASK_ADC; // enable bus clock for ADC
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(ADC_GCLK_ID) |	GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0); // GCLK0 to ADC
	
    REG_ADC_SAMPCTRL = 10;       // sampling time 10 clocks
      
    ARRAY_PORT_PINCFG0[3] |= 1; // Use PMUX for PA03
    ARRAY_PORT_PMUX0[1] = 0x10; // PA03 = VREFA
    REG_ADC_REFCTRL = 3;        // Use VREFA
    
    //REG_ADC_REFCTRL = 0x80 | 0x02; // Reference buffer offset compensation is enabled; Reference Selection=VDDANA/2
    REG_ADC_CTRLB |= 0x0700; // clock pre-scaler is: Peripheral clock divided by 512
    
    REG_ADC_INPUTCTRL = 0x1805; // V- = GND; V+ = AIN5
    ARRAY_PORT_PINCFG0[4] |= 1; // Use PMUX for PA04
    ARRAY_PORT_PINCFG0[5] |= 1; // Use PMUX for PA05
    ARRAY_PORT_PMUX0[2] = 0x11; // PA04 = AIN4, PA05 = AIN5 (low nibble is for AIN4, high nibble is for AIN5)
    
    REG_ADC_CALIB = ADC_CALIB_BIAS_CAL(NVM_READ_CAL(ADC_BIASCAL)) |
                    ADC_CALIB_LINEARITY_CAL(NVM_READ_CAL(ADC_LINEARITY));

    REG_ADC_CTRLA = 2;          // enable ADC
}

int ADC_read (unsigned int channel)
{
    int result;

    REG_ADC_INPUTCTRL = 0x1800 | channel; // V- = GND; V+ = channel (either 4 or 5 as configured above)
    
    REG_ADC_SWTRIG = 2;             // start a conversion
    while(!(REG_ADC_INTFLAG & 1));  // wait for conversion complete
    result = REG_ADC_RESULT;        // read conversion result
    
    return result;
}

void ConfigurePins (void)
{
	// Configure input pins
    REG_PORT_DIRCLR0 = PORT_PA15; // Period surbroutine input pin
    ARRAY_PORT_PINCFG0[15] |= 6;  // enable PA15 input buffer with pull
    REG_PORT_OUTSET0 = PORT_PA15; // PA15 pull-up
    
    // Configure output pins
    REG_PORT_DIRSET0 = PORT_PA00; // Configure PA00 as output.  This is pin 1 of the LQFP32 package.
    REG_PORT_DIRSET0 = PORT_PA01; // Configure PA01 as output.  This is pin 2 of the LQFP32 package.
    REG_PORT_DIRSET0 = PORT_PA02; // Configure PA02 as output.  This is pin 3 of the LQFP32 package.
    REG_PORT_DIRSET0 = PORT_PA06; // Configure PA06 as output.  This is pin 7 of the LQFP32 package.
    REG_PORT_DIRSET0 = PORT_PA07; // Configure PA07 as output.  This is pin 8 of the LQFP32 package.
    REG_PORT_DIRSET0 = PORT_PA08; // Configure PA08 as output.  This is pin 11 of the LQFP32 package.
    REG_PORT_DIRSET0 = PORT_PA09; // Configure PA09 as output.  This is pin 12 of the LQFP32 package.
}

//BLOCK DIAGRAM: https://docs.google.com/drawings/d/1T3Urse6BNoc9c-jF8palLSMFl9NvYzkgg40_ldZfCOA/edit
int main(void)
{
	void turn_around();
	int get_perimeter_reading();
	int detect_coin();
	void move_forward();
	void operate_arm();
	
	int adc_value;
	unsigned long v;
	unsigned long int count, f;
	unsigned char LED_toggle=0;
	
	init_Clock48();
    UART3_init(115200);
    ADC_init();
    ConfigurePins();
	Configure_TC2_servo();
    Configure_TC3();
	Configure_TC3_servo();

	UART3_init(115200);

	//Robot operation starts here, refer to block diagram comment above for full explanation
	
	int coins = 0;

	while(coins < 20) {
		if(get_perimeter_reading()){
			turn_around();
		}
		else if(detect_coin()){
			coin_pickup_reverse();
			operate_arm();
			coins++;
		}
		else {
			move_forward();
		}
	}
    
	return 1;
}

/*-----------------------------------------------------------------
//checks for perimeter wire, if detected, returns 1
//input: none
//output: 1 if wire detected, else 0
-----------------------------------------------------------------*/
int get_perimeter_reading(){

}

/*-----------------------------------------------------------------
//checks for coin, if detected, returns 1
//input: none
//output: 1 if coin detected, else 0
-----------------------------------------------------------------*/
int detect_coin(){

}


/*-----------------------------------------------------------------
//moves robot forward by small amount
//input: none
//output: none
-----------------------------------------------------------------*/
void move_forward(){
	
}

/*-----------------------------------------------------------------
//does a 90 degree turn if the perimeter wire is detected
//input: none
//output: none
-----------------------------------------------------------------*/
void turn_around(){

}

/*-----------------------------------------------------------------
//operate the servos to pick up the coin
//input: none
//output: none
-----------------------------------------------------------------*/
void coin_pickup() {
	int x, y
	x = 60;
	y = 60;
	servo(x, y);
}

/*-----------------------------------------------------------------
//moves servos to position specified by parameter n and m *pin 16 & pin 18
//input: 	n - TC2 Servo
			m - TC3 Servo
//output: none
-----------------------------------------------------------------*/
void servo(int n, int m){
	REG_TC2_COUNT16_CC1 = (((F_CPU/(64*50))* n )/2000)-1;
	REG_TC3_COUNT16_CC1 = (((F_CPU/(64*50))* m )/2000)-1;
}

