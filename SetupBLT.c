// SetupBLT.c
// Runs on TM4C123
// This is an example program to setup HC-05 Bluetooth module with no user interface.
// UART0 is used for the TM4C123 to communicate with PC serial terminal, 
// UART1 is used for the TM4C123 to cummunicate with HC-05 Bluetooth module
// U1Rx (PB0) connects to HC-05 TXD pin
// U1Tx (PB1) connects to HC-05 RXD pin
// HC-05 VCC connects to vbus pin on TM4C123
// HC-05 EN connects to +3.3v
// By Min He,10/11/2022

#include "tm4c123gh6pm.h"
#include "UART0.h"
#include "BLT.h"
#include <stdio.h>
#include <stdint.h>  // for data type alias

//AT+ORGL restore default settings.
// main function for programming BT device with no UI
int main(void) {
	uint8_t String[30];
	uint8_t command[30];
	// String to concatenate
  char append[] = "\r\n";
	uint8_t i;
	UART0_Init();
	UART0_OutString((unsigned char *)">>> Welcome to Serial Terminal. <<<\n\r");
	UART0_OutString((unsigned char *)">>> This is the setup program for the HC-05 bluetooth module. <<<\n\r");
	UART0_OutString((unsigned char *)">>> You are in 'AT' command. <<<\n\r");
	UART0_OutString((unsigned char *)">>> Type 'AT' followed by a command. <<<\n\r");
	UART0_OutString((unsigned char *)">>> Example: AT+NAME=Your Name  <<<\n\r");
	BLT_Init();
	//uint8_t SetCommands[][30] = {"AT+NAME=Banana\r\n","AT+UART=57600,0,1\r\n","AT+PSWD=0824\r\n","AT+ROLE=0\r\n"};
	//uint8_t QueryCommands[][30] = {"AT+NAME?\r\n","AT+UART?\r\n","AT+PSWD?\r\n","AT+ROLE?\r\n"};
	while(1){
		UART0_InString(command, 30); // Type command.
		UART0_NextLine();           //Creates new line.
		//if(command[2]=='+' && command[7] == '=' || command[2]=='+' && command[7] == '?'){ // Setup the HC-05 bluetooth module by checking correct parameters.
		//	if(command[6]=='E' || command[6] == 'T' || command[6] == 'D'){
				//UART0_OutString(command);    //Return command that was typed.
				sprintf((char *)command, "%s%s", (char *)command, append);
				BLT_OutString(command);
			  while ((UART1_FR_R&UART_FR_BUSY) != 0){};
        BLT_InString(String);
				UART0_OutChar(SP);
        UART0_OutString(String); //Displays OK
        UART0_NextLine();	
				command[0] = '\0';
				String[0] = '\0';
			//}
	//	}else
		//	UART0_OutString((unsigned char *)"Invalid Command\n\r");
	}
}
