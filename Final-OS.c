//Transmit bytes with UART
#include <msp430fr6989.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define redLED BIT0 // Red LED at P1.0
#define greenLED BIT7 // Green at P9.7
#define BUT1 BIT1 // Button S1 at Port 1.1
#define BUT2 BIT2 // Button S2 at Port 1.2
#define FLAGS UCA1IFG // Contains the transmit & receive flags
#define RXFLAG UCRXIFG // Receive flag
#define TXFLAG UCTXIFG // Transmit flag
#define TXBUFFER UCA1TXBUF // Transmit buffer
#define RXBUFFER UCA1RXBUF // Receive buffer
#define HISTORY_SIZE 5 //The amount of commands that can be stored in history

//Globals
char command[50]; //Create char array to hold command
char history[HISTORY_SIZE][50]; //Create array of strings for history
int historysize = 0; //keeps track of number of entries in history

void Initialize_ADC() {

    // Divert the pins to analog functionality
    // X-axis: A10/P9.2, for A10 (P9DIR=x, P9SEL1=1, P9SEL0=1)
    P9SEL1 |= BIT2;
    P9SEL0 |= BIT2;

    //Y-axis: A4/P8.7, for A4 (P9DIR=x, P9SEL1=1, P9SEL0=1)
    P8SEL1 |= BIT7;
    P8SEL0 |= BIT7;

    // Turn on the ADC module
    ADC12CTL0 |= ADC12ON;

    // Turn off ENC (Enable Conversion) bit while modifying the configuration
    ADC12CTL0 &= ~ADC12ENC;

    //*************** ADC12CTL0 ***************
    // Set ADC12SHT0 (select the number of cycles that you determined)
    ADC12CTL0 |= ADC12SHT0_3; //Configure to 32 cycles

    // Set the bit ADC12MSC (Multiple Sample and Conversion)
    ADC12CTL0 |= ADC12MSC;

    //*************** ADC12CTL1 ***************
    // Set ADC12SHS (select ADC12SC bit as the trigger)
    // Set ADC12SHP bit
    // Set ADC12DIV (select the divider you determined)
    // Set ADC12SSEL (select MODOSC)
    ADC12CTL1 = ADC12SHP | ADC12SSEL_0 | ADC12SHS_0 | ADC12DIV_0;

    // Set ADC12CONSEQ (select sequence-of-channels
    ADC12CTL1 |= ADC12CONSEQ_1;

    //*************** ADC12CTL2 ***************
    // Set ADC12RES (select 12-bit resolution)
    // Set ADC12DF (select unsigned binary format)
    ADC12CTL2 = ADC12RES_2;

    //*************** ADC12CTL3 ***************
    // Set ADC12CSTARTADD to 0 (first conversion in ADC12MEM0)
    ADC12CTL3 = ADC12CSTARTADD_0;

    //*************** ADC12MCTL0 ***************
    // Set ADC12VRSEL (select VR+=AVCC, VR-=AVSS)
    // Set ADC12INCH (select channel A10)
    ADC12MCTL0 |= ADC12INCH_10 | ADC12VRSEL_0;

    //*************** ADC12MCTL1 ***************
    // Set ADC12VRSEL (select VR+=AVCC, VR-=AVSS)
    // Set ADC12INCH (select the analog channel that you found)
    // Set ADC12EOS (last conversion in ADC12MEM1)
    ADC12MCTL1 |= ADC12INCH_4 | ADC12VRSEL_0 | ADC12EOS;


    // Turn on ENC (Enable Conversion) bit at the end of the configuration
    ADC12CTL0 |= ADC12ENC;

    return;
}

// Configure eUSCI in I2C master mode
void Initialize_I2C(void) {

          // Enter reset state before the configuration starts...
          UCB1CTLW0 |= UCSWRST;

          // Divert pins to I2C functionality
          P4SEL1 |= (BIT1|BIT0);
          P4SEL0 &=  ~(BIT1|BIT0);

          // Keep all the default values except the fields below...
          // (UCMode 3:I2C) (Master Mode) (UCSSEL 1:ACLK, 2,3:SMCLK)
          UCB1CTLW0 |= UCMODE_3 | UCMST | UCSSEL_3;

          // Clock divider = 8 (SMCLK @ 1.048 MHz / 8 = 131 KHz)
          UCB1BRW = 8;

          // Exit the reset mode
          UCB1CTLW0 &=  ~UCSWRST;
}

// Read a word (2 bytes) from I2C (address, register)
int i2c_read_word(unsigned char i2c_address, unsigned char i2c_reg, unsigned int * data) {

    unsigned char byte1, byte2;

    // Initialize the bytes to make sure data is received every time
    byte1 = 111;
    byte2 = 111;

    //********** Write Frame #1 ***************************
    UCB1I2CSA = i2c_address; // Set I2C address
    UCB1IFG &= ~UCTXIFG0;
    UCB1CTLW0 |= UCTR; // Master writes (R/W bit = Write)
    UCB1CTLW0 |= UCTXSTT; // Initiate the Start Signal

    while ((UCB1IFG & UCTXIFG0) ==0) {}

    UCB1TXBUF = i2c_reg; // Byte = register address

    while((UCB1CTLW0 & UCTXSTT)!=0) {}

    if(( UCB1IFG & UCNACKIFG )!=0) return -1;

    UCB1CTLW0 &= ~UCTR; // Master reads (R/W bit = Read)
    UCB1CTLW0 |= UCTXSTT; // Initiate a repeated Start Signal
    //****************************************************

    //********** Read Frame #1 ***************************
    while ( (UCB1IFG & UCRXIFG0) == 0) {}
    byte1 = UCB1RXBUF;

    //****************************************************

    //********** Read Frame #2 ***************************
    while((UCB1CTLW0 & UCTXSTT)!=0) {}
    UCB1CTLW0 |= UCTXSTP; // Setup the Stop Signal

    while ( (UCB1IFG & UCRXIFG0) == 0) {}
    byte2 = UCB1RXBUF;

    while ( (UCB1CTLW0 & UCTXSTP) != 0) {}
    //****************************************************

    // Merge the two received bytes
    *data = ( (byte1 << 8) | (byte2 & 0xFF) );
    return 0;
}

// Write a word (2 bytes) to I2C (address, register)
int i2c_write_word(unsigned char i2c_address, unsigned char i2c_reg, unsigned int data) {

    unsigned char byte1, byte2;

    byte1 = (data >> 8) & 0xFF; // MSByte
    byte2 = data & 0xFF; // LSByte

    UCB1I2CSA = i2c_address; // Set I2C address

    UCB1CTLW0 |= UCTR; // Master writes (R/W bit = Write)
    UCB1CTLW0 |= UCTXSTT; // Initiate the Start Signal

    while ((UCB1IFG & UCTXIFG0) ==0) {}

    UCB1TXBUF = i2c_reg; // Byte = register address

    while((UCB1CTLW0 & UCTXSTT)!=0) {}

    //********** Write Byte #1 ***************************
    UCB1TXBUF = byte1;
    while ( (UCB1IFG & UCTXIFG0) == 0) {}

    //********** Write Byte #2 ***************************
    UCB1TXBUF = byte2;
    while ( (UCB1IFG & UCTXIFG0) == 0) {}

    UCB1CTLW0 |= UCTXSTP;
    while ( (UCB1CTLW0 & UCTXSTP) != 0) {}

    return 0;
}

void uart_write_char(unsigned char ch){

    // Wait for any ongoing transmission to complete
    while ((FLAGS & TXFLAG)==0 ) {}

    // Write the byte to the transmit buffer
    TXBUFFER = ch;
}

// The function returns the byte; if none received, returns NULL
unsigned char uart_read_char(){

    unsigned char temp;
    char tempstring[1];

    // Return NULL if no byte received
    if((FLAGS & RXFLAG) == 0) return NULL;

    // Otherwise, copy the received byte (clears the flag) and return it
    temp = RXBUFFER;

    //Concat the char if it not backspace or enter
    if((temp != '\r') && (temp != '\b')) {
        sprintf(tempstring, "%c", temp);
        strcat(command, tempstring);
    }

    return temp;
}

//Basically same function except its used for the echo function so doesent need to store command
unsigned char uart_echo_char(){

    unsigned char temp;

    // Return NULL if no byte received
    if((FLAGS & RXFLAG) == 0) return NULL;

    // Otherwise, copy the received byte (clears the flag) and return it
    temp = RXBUFFER;

    return temp;
}

void uart_write_string(char * str) {

    char letter = 'a'; //Set to random letter
    unsigned int k = 0; //Increment

    while(letter != '\0') { //Go until end of string
        letter = str[k];
        uart_write_char(letter);
        k++;
    }
}

// Configure UART to the popular configuration
// 9600 baud, 8-bit data, LSB first, no parity bits, 1 stop bit, no flow control
// Initial clock: SMCLK @ 1.0 MHz with oversampling
void Initialize_UART(void){
    // Divert pins to UART functionality
    P3SEL1 &=  ~(BIT4|BIT5);
    P3SEL0 |= (BIT4|BIT5);
    UCA1CTLW0 = UCSWRST;

    // Use SMCLK clock; leave other settings default
    UCA1CTLW0 |= UCSSEL_2;

    // Configure the clock dividers and modulators
    // UCBR=6, UCBRF=8, UCBRS=0x20, UCOS16=1 (oversampling)
    UCA1BRW = 6;
    UCA1MCTLW = UCBRS5|UCBRF3|UCOS16;

    // Exit the reset state (so transmission/reception can begin)
    UCA1CTLW0 &=  ~UCSWRST;
}

void help() {

    uart_write_string("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\r");
    uart_write_string("The following commands can be entered:\n\r");
    uart_write_string("help - Shows all the possible commands\n\r");
    uart_write_string("LED [color] [mode] - The color argument should be \"red\" or \"green\". The mode argument should be \"blink\" or \"toggle\" \n\r");
    uart_write_string("history - This will display all previously entered commands. -c can be passed to clear the history\n\r");
    uart_write_string("light [mode] - This will print the lux value one time or continuously depending on the mode. Options are \"continuous\" and \"blink\"\n\r");
    uart_write_string("joystick [mode] - This will print the x and y coordinates of the joystick one time or continuously depending on the mode. Options are \"continuous\" and \"blink\"\n\r");
    uart_write_string("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\r");
}

void LED(char * color, char * mode) {

    //Check if mode is toggle
    if (strcmp(mode, "toggle") == 0) {

        //Check which color
        if(strcmp(color, "red") == 0) P1OUT ^= redLED;
        else if(strcmp(color, "green") == 0) P9OUT ^= greenLED;

        //Unrecognized color
        else uart_write_string("Unrecognized color see help for further details\n\r");

    }

    //Check if mode is blink
    else if (strcmp(mode, "blink") == 0) {

        //Check which color
        if(strcmp(color, "red") == 0) {

            // Configure channel 0 timer 0 interrupt for red LED
            TA0CCR0 = 32787;
            TA0CCTL0 |= CCIE; //Enable interrupts in channel 0
            TA0CCTL0 &= ~CCIFG; //reset flag

            //Configure timer_A to ACLK, up mode, and divide by 8
            TA0CTL = TASSEL_1 | ID_3 | MC_1 | TACLR;

            //Turn on LED
            P1OUT |= redLED;
        }
        else if(strcmp(color, "green") == 0) P9OUT |= greenLED;

        //Unrecognized color
        else uart_write_string("Unrecognized color see help for further details\n\r");
    }

    //Unrecognized mode
    else uart_write_string("Unrecognized mode see help for further details\n\r");
}

void light(char * mode) {

    unsigned int config = 0; //Depending on the mode we will set this to a different hex value
    char string[25]; //For sprintf
    volatile uint32_t delay; //Create delay variable

    //Check if mode is blink
    if(strcmp(mode, "blink") == 0) {

        //Set hex value and write to config register (0x01) of light sensor (0x44)
        config = 0x7204;
        i2c_write_word(0x44, 0x01, config);

        //Read the light sensors result register (0x00)
        i2c_read_word(0x44, 0x00, &config);
        config *= 1.28;

        //Convert int to string
        sprintf(string, "Lux value = %d\n\r", config);
        uart_write_string(string);

    }

    //Check if mode is continuous
    else if(strcmp(mode, "continuous") == 0) {

        //Set hex value and write to config register (0x01) of light sensor (0x44)
        config = 0x7604;
        i2c_write_word(0x44, 0x01, config);
        uart_write_string("Press left button to stop:\n\r");

        //Loop until button is pressed
        while ((P1IN & BUT1) != 0) {

             //Read the light sensors result register (0x00)
             i2c_read_word(0x44, 0x00, &config);
             config *= 1.28;

             //Convert int to string
             sprintf(string, "Lux value = %d\n\r", config);
             uart_write_string(string);

             ////Big delay loop
             for(delay=0; delay<=100000; delay++) {}
        }
    }

    //Unrecognized mode
    else uart_write_string("Unrecognized mode see help for further details\n\r");

}

void joystick(char * mode) {

    char string[25]; //For sprintf
    volatile uint32_t delay; //Create delay variable

    //Check if mode is blink
    if(strcmp(mode, "blink") == 0) {

        //Start Conversion
        ADC12CTL0 |= ADC12SC;

        //Wait until busy flag is set
        while((ADC12CTL1 & ADC12BUSY) != 0) {}

        //Convert cords to string
        sprintf(string, "X-axis = %d\n\rY-axis = %d\n\r", ADC12MEM0, ADC12MEM1);
        uart_write_string(string);
    }

    //Check if mode is continuous
    else if(strcmp(mode, "continuous") == 0) {

        uart_write_string("Press left button to stop:\n\r");

        //Loop until button is pressed
        while ((P1IN & BUT1) != 0) {

            //Start Conversion
            ADC12CTL0 |= ADC12SC;

            //Wait until busy flag is set
            while((ADC12CTL1 & ADC12BUSY) != 0) {}

            //Convert cords to string
            sprintf(string, "X-axis = %d\n\rY-axis = %d\n\r\n", ADC12MEM0, ADC12MEM1);
            uart_write_string(string);

            //Big delay loop
            for(delay=0; delay<=100000; delay++) {}

        }
    }

    //Unrecognized mode
    else uart_write_string("Unrecognized mode see help for further details\n\r");
}

void addtohistory(char * command) {

    int i; //for iterating

    //If history is not full
    if(historysize != HISTORY_SIZE) {
        strcpy(history[historysize], command);
        historysize++;
    }

    else {
        //Iterate through history moving each element down
        for(i = 1; i < HISTORY_SIZE; i++) strcpy(history[i-1], history[i]);

        //Now just add the new command
        strcpy(history[historysize-1], command);
    }
}

int main(void) {

    WDTCTL = WDTPW | WDTHOLD; // Stop WDT
    PM5CTL0 &=  ~LOCKLPM5; // Enable GPIO pins

    unsigned char byte; //Create variable to read key press
    volatile uint32_t delay; //Create delay variable

    Initialize_UART(); //Set up UART
    Initialize_I2C(); //Set up I2C
    Initialize_ADC(); //Set up digital converter

    //Setup LEDS to off
    P1DIR |= redLED;
    P1OUT &=  ~redLED;
    P9DIR |= greenLED;
    P9OUT &=  ~greenLED;

    //Configure buttons
    P1DIR &=  ~(BUT1|BUT2);
    P1REN |= (BUT1|BUT2);
    P1OUT |= (BUT1|BUT2);

    // Enable GIE bit for interrupts
    _enable_interrupts();

    //Infinite loop
    for(;;) {

        //Slight delay loop
        for(delay=0; delay<=5000; delay++) {}

        //Read input
        byte = uart_read_char();

        //Write back whatever was typed
        uart_write_char(byte);

        //Enter key has been pressed to send a command
        if(byte == '\r'){
            uart_write_char('\n');

            //Add command to history
            addtohistory(command);

            //Get first argument
            char * argument;
            argument = strtok(command, " ");

            //Check which command was entered
            if(strcmp(argument, "help") == 0) help();

            else if(strcmp(argument, "LED") == 0) {

                char color[50];
                char mode[50];

                //Grab next argument
                argument = strtok(NULL, " ");
                strcpy(color, argument);

                //Only proceed if there actually was an argument
                if(argument != NULL) {

                    //Grab next argument
                    argument = strtok(NULL, " ");
                    strcpy(mode, argument);

                    //Call function
                    if(argument != NULL) LED(color, mode);
                }
                if(argument == NULL) uart_write_string("Not enough arguments\n\r");
            }

            else if(strcmp(argument, "history") == 0) {

                //Grab next argument
                argument = strtok(NULL, " ");

                //Check if -c was passed
                if(strcmp(argument, "-c") == 0) historysize = 0;

                //This means just history was used
                else if(argument == NULL) {

                    //Print each previous command
                    int i = 0;
                    uart_write_string("Current History:\n\r");
                    for(i = 0; i < historysize; i++) {
                        uart_write_string(history[i]);
                        uart_write_string("\r\n");
                    }
                }
                else uart_write_string("Invalid Argument\n\r");
            }

            else if(strcmp(argument, "light") == 0) {

                //Grab next argument
                argument = strtok(NULL, " ");

                if(argument == NULL) uart_write_string("Not enough arguments\n\r");
                else light(argument);
            }

            else if(strcmp(argument, "joystick") == 0) {

                //Grab next argument
                argument = strtok(NULL, " ");

                if(argument == NULL) uart_write_string("Not enough arguments\n\r");
                else joystick(argument);
            }


            else uart_write_string("Command is not recognized\n\r");

            //Clear command
            command[0] = '\0';

        }

        //Backspace key has been pressed
        else if(byte == '\b') {
            //Delete character on terminal
            uart_write_char(' ');
            uart_write_char('\b');

            //Delete character from command
            command[strlen(command) - 1] = '\0';
        }

    }
}

//This interrupt occurs for red LED
#pragma vector = TIMER0_A0_VECTOR
__interrupt void red_ISR() {

    //Disable LED
    P1OUT &= ~redLED;

    //Disable the interrupt now
    TA0CCTL0 &= ~CCIE;

    //Dont need to reset flag since channel 0
}
