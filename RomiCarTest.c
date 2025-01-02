// BLTControlledLEDs.c
// Runs on LM4F120/TM4C123
// This is a program which shows how to use Bluetooth to receive commands from a Bluetooth terminal
// and use the received command to control the onboard LEDs, as well as to control DC motors using PWM.
// U1Rx (PB0)
// U1Tx (PB1)
// Ground connected ground in the USB cable

// Header files
#include "tm4c123gh6pm.h"
#include <stdint.h>
#include"PWM.h"
#include <stdbool.h>


#define LED GPIO_PORTF_DATA_R
#define DIRECTION (*((volatile unsigned long *)0x4002403C)) // Mask for PE0-3
#define FORWARD 		0x0F		// 1111
#define BACKWARD 		0x0A		// 1010
#define LEFT  		  0x0E    // 1100
#define RIGHT  		  0x0B    // 1011
#define NVIC_EN0_PORTF 0x40000000


#define Dark 0x00
#define Red 0x02
#define Blue 0x04
#define Green 0x08
#define Yellow 0x0A
#define White 0x0E
#define Purple 0x06
#define SW1       0x10
#define SW2       0x01

// standard ASCII symbols
#define CR 0x0D
#define LF 0x0A

#define PERIOD 10000 // Total PWM period
#define STOP 1 // min duty cycle (0%)
#define SPEED_10 1000 // 10% duty cycle
#define SPEED_1010 1010 // 10.10% duty cycle
#define SPEED_37 3700 // 35% duty cycle
#define SPEED_20 2000 // 20% duty cycle
#define SPEED_30 3000 // 35% duty cycle
#define SPEED_40 4000 //40%
#define SPEED_45 4500 //45%
#define SPEED_50 5000 //50%
#define SPEED_60 6000 //60%
#define SPEED_70 7000 //70%
#define SPEED_80 8000 //80
#define SPEED_88 8800 //88%
#define SPEED_98 9800 //98

// Function prototypes
void UART_Init(void);
unsigned char UART1_InChar(void);
void UART0_OutChar(unsigned char data);
void UART0_OutString(unsigned char *pt);
void PortF_Init(void);
void Delay(void);
void Delay02(void);
void Switch_Init(void);

extern void DisableInterrupts(void); // Disable interrupts
extern void EnableInterrupts(void);  // Enable interrupts
extern void WaitForInterrupt(void);  // low power mode

bool mode1;

int main(void)
{
		DisableInterrupts();
    unsigned char control_symbol; // for Bluetooth controlled LEDs
	  mode1 = true;
	  uint16_t current_speed = SPEED_30;
	  uint16_t diff_speed;
	  PWM0A_Init(PERIOD);         // initialize PWM0, PB6 LEFT MOTOR
		PWM0B_Init(PERIOD);         // initialize PWM0, PB7 RIGHT MOTOR
		PortE_Init();								// initialize PE0-3, direction - PE0&1 for R, PE2&3 for L
	  //PortF_Init();                 // Initialize for the three onboard LEDs
	  Switch_Init();
	  UART_Init();                  // Initialize UART1
		EnableInterrupts();
    UART0_OutString((unsigned char *)">>> Welcome to Bluetooth Controlled LED and DC Motor App <<<\n\r");
	  LED=Green;


    // Bluetooth Controlled LEDs and DC Motors
    while (1)
    {
			
			if(mode1){
				current_speed = SPEED_30;
        control_symbol = UART1_InChar();
        UART0_OutChar(control_symbol);
        UART0_OutChar(CR);
        UART0_OutChar(LF);
				
        switch (control_symbol)
        {
        case 'C':
        case 'c':
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);
						if(current_speed==SPEED_30){
							diff_speed=SPEED_20;
						}
						else if(current_speed==SPEED_40){
							diff_speed=SPEED_30;
						}
						else if(current_speed==SPEED_50){
							diff_speed=SPEED_40;
						}
						else if(current_speed==SPEED_60){
							diff_speed=SPEED_50;
						}
            DIRECTION = FORWARD;
						PWM0A_Duty(current_speed);
						PWM0B_Duty(diff_speed);
				    Delay();
				    Delay();
				    Delay();
				    Delay();
				    Delay();
				    Delay();
				    Delay();
				    Delay();
				    Delay();
				    Delay();
				    Delay();
  			    Delay();
  			    Delay();
  			    Delay();
//				    Delay();
//				    Delay();
//				    Delay();
//				    Delay();
//						Delay();
//				    Delay();
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);
            break;
        case 'S'://square
        case 's':
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);
				    DIRECTION = FORWARD;
						PWM0A_Duty(current_speed);
						PWM0B_Duty(current_speed);
				    Delay();
						Delay();
						Delay();
						Delay();
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);
				
						DIRECTION=LEFT;
						PWM0A_Duty(current_speed);
						PWM0B_Duty(SPEED_10);
						Delay();	
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);	

				    DIRECTION = FORWARD;
						PWM0A_Duty(current_speed);
						PWM0B_Duty(current_speed);
				    Delay();
						Delay();
						Delay();
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);	

						DIRECTION=LEFT;
						PWM0A_Duty(current_speed);
						PWM0B_Duty(SPEED_10);
						Delay();	
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);	

						
				    DIRECTION = FORWARD;
						PWM0A_Duty(current_speed);
						PWM0B_Duty(current_speed);
				    Delay();
						Delay();
						Delay();
						Delay();
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);	
						
						DIRECTION=LEFT;
						PWM0A_Duty(current_speed);
						PWM0B_Duty(SPEED_10);
						Delay();	
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);	
						
						
				    DIRECTION = FORWARD;
						PWM0A_Duty(current_speed);
						PWM0B_Duty(current_speed);
				    Delay();
						Delay();
						Delay();
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);	
						
						DIRECTION=LEFT;
						PWM0A_Duty(current_speed);
						PWM0B_Duty(SPEED_10);
						Delay();	
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);	
						
				    DIRECTION = FORWARD;
						PWM0A_Duty(current_speed);
						PWM0B_Duty(current_speed);
				    Delay();
						Delay();
						Delay();
						Delay();
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);	
            break;
        case '8':
					  diff_speed=SPEED_20;
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);
            DIRECTION = FORWARD; //Turns a bit in circle to the left
						PWM0A_Duty(current_speed);
						PWM0B_Duty(diff_speed);
				    Delay();
				    Delay();
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);	

						DIRECTION = FORWARD; //goes forward
						PWM0A_Duty(current_speed);
						PWM0B_Duty(current_speed);
				    Delay();
				    Delay();
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);	
				
						DIRECTION=RIGHT; //Turns 90 degree right
						PWM0A_Duty(SPEED_10);
						PWM0B_Duty(current_speed);
						Delay();	
						Delay02();
						Delay02();
						Delay02();
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);	

						DIRECTION = FORWARD; //goes forward
						PWM0A_Duty(current_speed);
						PWM0B_Duty(current_speed);
				    Delay();
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);	
						
						DIRECTION=RIGHT; //Turns a bit to the right
						PWM0A_Duty(SPEED_10);
						PWM0B_Duty(current_speed);
						Delay();	
						Delay02();
						Delay02();
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);	
						
						DIRECTION = FORWARD; //goes forward
						PWM0A_Duty(current_speed);
						PWM0B_Duty(current_speed);
				    Delay();
				    Delay();
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);	
						
						DIRECTION=LEFT; //Turns 90 degree left
						PWM0A_Duty(current_speed);
						PWM0B_Duty(SPEED_10);
						Delay();	
						Delay02();
						Delay02();
						Delay02();
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);	
						
						DIRECTION = FORWARD; //goes forward
						PWM0A_Duty(current_speed);
						PWM0B_Duty(current_speed);
				    Delay();
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);	
            break;
        default:
            break;
        }
			}
			else{
				control_symbol = UART1_InChar();
        UART0_OutChar(control_symbol);
        UART0_OutChar(CR);
        UART0_OutChar(LF);
        switch (control_symbol)
        {
        case 'F':
        case 'f':
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);
            DIRECTION = FORWARD;
						PWM0A_Duty(current_speed);
						PWM0B_Duty(current_speed);
            break;
        case 'B':
        case 'b':
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);
            DIRECTION = BACKWARD;
						PWM0A_Duty(current_speed);
						PWM0B_Duty(current_speed);
            break;
        case 'L':
        case 'l':
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);
            DIRECTION = LEFT;
						PWM0A_Duty(current_speed);
						PWM0B_Duty(current_speed);
            break;
        case 'R':
        case 'r':
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);
            DIRECTION = RIGHT;
						PWM0A_Duty(current_speed);
						PWM0B_Duty(current_speed);
            break;
        case 'S':
        case 's':
						PWM0A_Duty(STOP);
						PWM0B_Duty(STOP);
            break;
				case 'U':
				case 'u':
						if(current_speed==SPEED_30){
							current_speed=SPEED_40;
							PWM0A_Duty(STOP);
							PWM0B_Duty(STOP);
							PWM0A_Duty(current_speed);
							PWM0B_Duty(current_speed);
						}
						else if(current_speed==SPEED_40){
							current_speed=SPEED_50;
							PWM0A_Duty(STOP);
							PWM0B_Duty(STOP);
							PWM0A_Duty(current_speed);
							PWM0B_Duty(current_speed);
						}
						else if(current_speed==SPEED_50){
							current_speed=SPEED_60;
							PWM0A_Duty(STOP);
							PWM0B_Duty(STOP);
							PWM0A_Duty(current_speed);
							PWM0B_Duty(current_speed);
						}
						break;
				case 'D':
				case 'd':
						if(current_speed==SPEED_60){
							current_speed=SPEED_50;
							PWM0A_Duty(STOP);
							PWM0B_Duty(STOP);
							PWM0A_Duty(current_speed);
							PWM0B_Duty(current_speed);
						}
						else if(current_speed==SPEED_50){
							current_speed=SPEED_40;
							PWM0A_Duty(STOP);
							PWM0B_Duty(STOP);
							PWM0A_Duty(current_speed);
							PWM0B_Duty(current_speed);
						}
						else if(current_speed==SPEED_40){
							current_speed=SPEED_30;
							PWM0A_Duty(STOP);
							PWM0B_Duty(STOP);
							PWM0A_Duty(current_speed);
							PWM0B_Duty(current_speed);
						}
						break;
					
        default:
            break;
        }
			}
    }
}

void GPIOPortF_Handler(void)
{		
	// simple debouncing code: generate 20ms to 30ms delay
	for (uint32_t time=0;time<80000;time++) {}
	
  if(GPIO_PORTF_RIS_R & SW2) //When SW2 is pressed on Microcontroller 1 or 2, the led will become red.
	{
		GPIO_PORTF_ICR_R = SW2;	
		LED=Blue;
		mode1=false;
	}
	
	if(GPIO_PORTF_RIS_R & SW1)//If I pressed SW2 on microcontroller 1 and not 2, I can press sw1 to make microcontroller 2 red and vice versa.
	{
		GPIO_PORTF_ICR_R = SW1;
		LED=Green;
		mode1=true;
	}
}


// Port F Initialization
void PortF_Init(void)
{
    volatile unsigned long delay;
    SYSCTL_RCGC2_R |= 0x00000020; // 1) F clock
    delay = SYSCTL_RCGC2_R;       // delay
    GPIO_PORTF_LOCK_R = 0x4C4F434B; // 2) unlock PortF PF0
    GPIO_PORTF_CR_R |= 0x0E;        // allow changes to PF3-PF1
    GPIO_PORTF_AMSEL_R &= ~0x0E;    // 3) disable analog function
    GPIO_PORTF_PCTL_R &= ~0x0000FFF0; // 4) GPIO clear bit PCTL
    GPIO_PORTF_DIR_R |= 0x0E;       // 6) PF1-PF3 output
    GPIO_PORTF_AFSEL_R &= ~0x0E;    // 7) no alternate function
    GPIO_PORTF_DEN_R |= 0x0E;       // 8) enable digital pins PF3-PF1
	
}

//------------UART_Init------------
// Initialize the UART for 19200 baud rate (assuming 16 MHz UART clock),
// 8 bit word length, no parity bits, one stop bit, FIFOs enabled
// Input: none
// Output: none
void UART_Init(void)
{
    // Activate Clocks
    SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART1; // activate UART1
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB; // activate port B
    SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0; // activate UART0
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA; // activate port A

    UART0_CTL_R &= ~UART_CTL_UARTEN; // disable UART
    UART0_IBRD_R = 17;               // IBRD = int(16,000,000 / (16 * 57600)) = int(17.3611111)
    UART0_FBRD_R = 23;               // FBRD = round(3611111 * 64) = 27
                                     // 8 bit word length (no parity bits, one stop bit, FIFOs)
    UART0_LCRH_R = (UART_LCRH_WLEN_8 | UART_LCRH_FEN);
    UART0_CTL_R |= 0x301; // enable UART for both Rx and Tx

    GPIO_PORTA_AFSEL_R |= 0x03;    // enable alt funct on PA1,PA0
    GPIO_PORTA_DEN_R |= 0x03;      // enable digital I/O on PA1,PA0
                                   // configure PA1,PA0 as UART0
    GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & 0xFFFFFF00) + 0x00000011;
    GPIO_PORTA_AMSEL_R &= ~0x03;   // disable analog functionality on PA1,PA0

    UART1_CTL_R &= ~UART_CTL_UARTEN; // disable UART

    // Data Communication Mode, Buad Rate = 57600
    UART1_IBRD_R = 17; // IBRD = int(16,000,000 / (16 * 57600)) = int(17.3611111)
    UART1_FBRD_R = 23;  // FBRD = round(3611111 * 64) = 27

                        // 8 bit word length (no parity bits, one stop bit, FIFOs
                        // 8 bit word length (no parity bits, one stop bit, FIFOs)
    UART1_LCRH_R = (UART_LCRH_WLEN_8 | UART_LCRH_FEN);
    UART1_CTL_R |= 0x301; // enable UART for both Rx and Tx

    GPIO_PORTB_AFSEL_R |= 0x03; // enable alt funct on PB1,PB0
    GPIO_PORTB_DEN_R |= 0x03;   // enable digital I/O on PB1,PB0
                                // configure PB1,PB0 as UART1
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFFFF00) + 0x00000011;
    GPIO_PORTB_AMSEL_R &= ~0x03; // disable analog functionality on PB1,PB0
}

//------------UART0_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none
void UART0_OutChar(unsigned char data)
{
    while ((UART0_FR_R & UART_FR_TXFF) != 0)
        ;
    UART0_DR_R = data;
}

//------------UART0_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void UART0_OutString(unsigned char *pt)
{
    while (*pt)
    {
        UART0_OutChar(*pt);
        pt++;
    }
}

//------------UART1_InChar------------
// Wait for new serial port input
// Input: none
// Output: ASCII code for key typed
unsigned char UART1_InChar(void)
{
    while ((UART1_FR_R & UART_FR_RXFE) != 0)
        ;
    return ((unsigned char)(UART1_DR_R & 0xFF));
}

// This function reads response from HC-05 Bluetooth module.
void BLT_InString(unsigned char *bufPt)
{
    unsigned char length = 0;
    bufPt[length] = UART1_InChar();

    // Two possible endings for a reply from HC-05: OK\r\n, ERROR:(0)\r\n
    while (bufPt[length] != LF)
    {
        length++;
        bufPt[length] = UART1_InChar();
    };

    // add null terminator
    length++;
    bufPt[length] = 0;
}


// Subroutine to wait 0.5 sec
// Inputs: None
// Outputs: None
void Delay(void)
{
    unsigned long volatile time;
    time = 727240 * 100 / 91; // 1 sec
    while (time)
    {
        time--;
    }
    for (time = 0; time < 1000; time = time + 10)
    {
    }
}

void Delay02(void)
{
    unsigned long volatile time;
    time = 145448 * 100 / 91; // 0.2 sec
    while (time)
    {
        time--;
    }
}

// Initilize port F and arm PF4, PF0 for falling edge interrupts
void Switch_Init(void){  
	unsigned long volatile delay;
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // (a) activate clock for port F
  delay = SYSCTL_RCGC2_R;
  GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; // unlock GPIO Port F
  GPIO_PORTF_CR_R |= 0x1F;         // allow changes to PF4,0
  GPIO_PORTF_DIR_R &= ~0x11;    // (c) make PF4,0 in (built-in button)
  GPIO_PORTF_DIR_R |= 0x0E;     // make PF1-3 out
  GPIO_PORTF_AFSEL_R &= ~0x11;  //     disable alt funct on PF4,0
  GPIO_PORTF_DEN_R |= 0x1F;     //     enable digital I/O on PF4,0
  GPIO_PORTF_PCTL_R &= ~0x000FFFFF; //  configure PF4,0 as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x1F;  //     disable analog functionality on PF4,0
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4,0
  GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4,PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4,PF0 is not both edges
  GPIO_PORTF_IEV_R |= 0x11;    //     PF4,PF0 rising edge event
  GPIO_PORTF_ICR_R = 0x11;      // (e) clear flags 4,0
  GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4,PF0
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF1FFFFF)|0x00400000; // (g) bits:23-21 for PORTF, set priority to 5
  NVIC_EN0_R |= 0x40000000;      // (h) enable interrupt 30 in NVIC
}
