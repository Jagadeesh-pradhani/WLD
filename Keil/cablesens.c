 #include<LPC17xx.h>
 #include<stdio.h>
 #include<rtl.h>

 #include"lcd.h"

#define PWM_PERIOD 5000 // Replace with the actual period of your PWM signal in microseconds
#define FLOW_RATE_MAX 30.0

 #define FLOW_SENSOR_PIN 23 // P0.23 for the Hall Effect flow sensor
#define LED_PIN 5         // P0.5 for an indicator LED (optional)
#define FREQ 12000000  // Assuming a 12MHz crystal frequency
#define BAUD_RATE 9600 // Set the baud rate for HC-05 (default is often 9600)
#define resistivity 1.68e-8
#define pi 3.14159
#define in_res 1000    //initial resistance

#define	Ref_Vtg		3.300
#define	Full_Scale	0xFFF

int sensorData = 0;
unsigned int i=0;
uint32_t pulseWidth;
char buffer[16];
float dutyCycle;
volatile uint32_t microseconds = 0;
unsigned long adc_temp;
float in_vtg, res;
unsigned char vtg[7],dval[7];

// SysTick handler to increment the microsecond counter
void SysTick_Handler(void) {
    microseconds++;
}

// Function to initialize the SysTick timer for microsecond counting
void initSysTick(void) {
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000000);  // Configure for 1 microsecond interval
}

// Function to return the current time in microseconds
uint32_t micros(void) {
    return microseconds + (SysTick->LOAD - SysTick->VAL) / SystemCoreClock * 1000000;
}



void UART0_Init() {
    // Enable power to UART0
    LPC_SC->PCONP |= (1 << 3);

    // Enable and set up the UART0 pins (TXD0 and RXD0)
    LPC_PINCON->PINSEL0 |= (1 << 4) | (1 << 6); // Select TXD0 and RXD0 for P0.2 and P0.3

    // Set the UART0 parameters
    LPC_UART0->LCR = 0x83; // 8 bits, 1 stop bit, Enable DLAB (to set baud rate)
    LPC_UART0->DLM = 0;    // High byte of divisor
    LPC_UART0->DLL = FREQ / (16 * BAUD_RATE); // Low byte of divisor
    LPC_UART0->LCR = 0x03; // Clear DLAB, 8 bits, 1 stop bit
}



void UART0_SendChar(char data) {
    while (!(LPC_UART0->LSR & (1 << 5))); // Wait until the Transmit Holding Register is empty
    LPC_UART0->THR = data; // Load the data to be sent
}

char UART0_ReceiveChar() {
    while (!(LPC_UART0->LSR & 0x01)); // Wait until the Receive Data Ready bit is set
    return LPC_UART0->RBR; // Return the received data
}





void delay_ms(uint32_t ms) {
    uint32_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 2000; j++); // Adjust this value for your system frequency
}




void _puts(unsigned char *buf1)
{
    unsigned int i=0;

    while(buf1[i]!='\0')
    {
        UART0_SendChar(buf1[i]);

		i++;

         
       }
    return;
}


void initializeFlowSensorPin() {
    LPC_PINCON->PINSEL1 &= ~(3 << 14);  // Configure pin as GPIO
    LPC_GPIO0->FIODIR &= ~(1 << FLOW_SENSOR_PIN);  // Set pin as input
}




uint32_t measurePulseWidth() {
    // Function to measure the pulse width of the PWM signal
    uint32_t startTime, endTime;

    // Wait for the rising edge of the PWM signal
    while (!(LPC_GPIO0->FIOPIN & (1 << FLOW_SENSOR_PIN)));
    startTime = micros();

    // Wait for the falling edge of the PWM signal
    while (LPC_GPIO0->FIOPIN & (1 << FLOW_SENSOR_PIN));
    endTime = micros();

    return endTime - startTime;
}

void flowsens()
{
		unsigned int i;
	unsigned char vtg[7],dval[7], va[10];

	int flowSensorValue;
	int pulseCount = 1;
	int flowRate;
	int flowRate1;
	float val;
		while(1)
	{
		//sensorData = (LPC_GPIO0->FIOPIN >> FLOW_SENSOR_PIN) & 1;
		pulseWidth = measurePulseWidth();
		dutyCycle = (float)pulseWidth / PWM_PERIOD;
		val =  dutyCycle * FLOW_RATE_MAX;
		 


		sprintf(buffer, "Flow: %f L/min", val);
		sprintf(vtg,"%f L/min",val);
		
		
		//UART0_SendChar(vtg);
			_puts(vtg);	
		for(i=0;i<2000;i++);
		
	temp1 = 0x80;
	lcd_com();
	delay_lcd(800);
	lcd_puts(&buffer[0]);
	//delay_ms(1000);
		
	}
}

int main(void)
{



	SystemInit();
	initSysTick();
	UART0_Init();
	
	LPC_SC->PCONP |= (1<<15); //Power for GPIO block
	LPC_PINCON->PINSEL1 &= ~(0x3 << 14); // Clear bits 14 and 15
	LPC_SC->PCONP |= (1 << 15);
    LPC_SC->PCLKSEL0 |= (1 << 10); // Set peripheral clock divider to 1 for GPIO

    // Configure P0.5 as output
    LPC_GPIO0->FIODIR |= (1 << LED_PIN);
	 LPC_PINCON->PINSEL1 &= ~(0x11<<14);
    LPC_PINCON->PINSEL1 |= 0x01<<14;	//P0.23 as AD0.0
	LPC_SC->PCONP |= (1<<12);			//enable the peripheral ADC
	
	initializeFlowSensorPin();

    lcd_init();

	
    SystemCoreClockUpdate();
	LPC_GPIO0->FIOPIN |= (1 << LED_PIN);
		
	while(1)
	{
		LPC_ADC->ADCR = (1<<0)|(1<<21)|(1<<24);//0x01200001;	//ADC0.0, start conversion and operational	
		for(i=0;i<2000;i++);			//delay for conversion
		while((adc_temp = LPC_ADC->ADGDR) == 0x80000000);	//wait till 'done' bit is 1, indicates conversion complete
		adc_temp = LPC_ADC->ADGDR;
		adc_temp >>= 4;
		adc_temp &= 0x00000FFF;			//12 bit ADC
		in_vtg = (((float)adc_temp * (float)Ref_Vtg))/((float)Full_Scale);	//calculating input analog voltage
			//convert the readings into string to display on LCD
		sprintf(dval,"%x",adc_temp);
		for(i=0;i<2000;i++);

		res = (float)(in_vtg*in_res)/(Ref_Vtg - in_vtg);
		sprintf(vtg,"%3.2fV",in_vtg);

		temp1 = 0x8A;
		lcd_com();
		delay_lcd(800);
		lcd_puts(&vtg[0]);

		temp1 = 0xCB;
		lcd_com();
		delay_lcd(800);
		lcd_puts(&dval[0]);

        for(i=0;i<200000;i++);
        for(i=0;i<7;i++)
        vtg[i] = dval[i] = 0x00;
        adc_temp = 0;
        in_vtg = 0;
	}
	

}

 



