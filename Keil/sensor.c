 #include<LPC17xx.h>
 #include<stdio.h>
 #include<rtl.h>

 #include"lcd.h"

#define PWM_PERIOD 5000 // Replace with the actual period of your PWM signal in microseconds
#define FLOW_RATE_MAX 30.0
#define FLOW_SENSOR_PIN 23 // P0.23 for the Hall Effect flow sensor
#define LED1_PIN 4         // P0.5 for an indicator LED (optional)
#define LED2_PIN 5
#define FREQ 12000000  // Assuming a 12MHz crystal frequency
#define BAUD_RATE 9600 // Set the baud rate for HC-05 (default is often 9600)

int sensorData = 0, i,count;
uint32_t pulseWidth;
char buffer[16] = {"hi"};
float dutyCycle,freq,water;
OS_TID tsk1, tsk2;




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


void delay(void) {
    unsigned int i;
    for (i = 0; i <= 5000000; i++)
        ;
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
	count = 0;
    // Wait for the rising edge of the PWM signal
	for (i = 0; i < 10; i++) {  // For example, measure over 1 second
        count += LPC_GPIO0->FIOPIN & (1 << FLOW_SENSOR_PIN);
        delay_ms(1);
    }

    return count;
}

void flowsens()
{
	float val;
	int i;
	

		//sensorData = (LPC_GPIO0->FIOPIN >> FLOW_SENSOR_PIN) & 1;
		count = measurePulseWidth();
	freq = (float)1000000/10;
	water = (float)(freq/count)*60;
	val = (float)water;
	/*
		dutyCycle = (float)pulseWidth / PWM_PERIOD;
		val =  dutyCycle * FLOW_RATE_MAX*60/1000;*/
		
		 


		sprintf(buffer, "Flow: %f L/min", (float)val);
		
		
		//UART0_SendChar(vtg);
			//_puts(buffer);	
		for(i=0;i<2000;i++);
		
	temp1 = 0x80;
	lcd_com();
	delay_lcd(800);
	lcd_puts(&buffer[0]);
	//delay_ms(1000);
		

}


__task void job2(void)
{
	
while(1)
	{
		
		LPC_GPIO0->FIOPIN ^= (1 << LED1_PIN);
		_puts(buffer);

		delay();
	}
	os_dly_wait(100);
	
}


__task void job1(void)
{
	os_tsk_create(job2, 10);
		while(1)
	{
		LPC_GPIO0->FIOPIN ^= (1 << LED2_PIN);	
			flowsens();
		delay();
	}
	
	os_dly_wait(100);

	
}

int main(void)
{

		int val = 0, i ;




	SystemInit();
	LPC_SC->PCONP |= (1 << 15);
    LPC_SC->PCLKSEL0 |= (1 << 10); // Set peripheral clock divider to 1 for GPIO
LPC_GPIO0->FIODIR |= (1 << LED1_PIN);
	LPC_GPIO0->FIODIR |= (1 << LED2_PIN);
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // Enable the cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	//initSysTick();
	UART0_Init();
	LPC_SC->PCONP |= (1<<15); //Power for GPIO block
	LPC_PINCON->PINSEL1 &= ~(0x3 << 14); // Clear bits 14 and 15
	initializeFlowSensorPin();
	lcd_init();
	SystemCoreClockUpdate();
	i=0;

 os_sys_init_prio(job1, 10);

}

 



