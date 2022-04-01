#include "samd20.h"
#include <stdlib.h>

void init_Clock48 (void);
void UART3_init(uint32_t baud);
void printString (char * s);
int getString (char * s, int echo, int max);

void Configure_TC2 ()
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


void Configure_TC3 ()
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

int main(void)
{
	char mystr[32];
    int n = 0;
	
	init_Clock48(); // Switch to 48MHz clock
	Configure_TC2();
	Configure_TC3();
    UART3_init(115200);

    printString("Servo PWM signals generator. Check pin PA15 and PA17 (pin 16 and 18 of QFP32)\r\n");

    while (1)
    {
	    printString("Pulse width 1 [60 to 240]: ");
	    getString(mystr, 1, sizeof(mystr));
	    n=atoi(mystr);
	    if((n>=60)&&(n<=240))
	    {
		    REG_TC3_COUNT16_CC1 = (((F_CPU/(64*50))*n)/2000)-1;
	    }
	    
	    printString("Pulse width 2 [60 to 240]: ");
	    getString(mystr, 1, sizeof(mystr));
	    n=atoi(mystr);
	    if((n>=60)&&(n<=240))
	    {
		    REG_TC2_COUNT16_CC1 = (((F_CPU/(64*50))*n)/2000)-1;
	    }
   }
}
