#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <math.h>        // math funcitons
//#include "NU32.h"       // constants, funcs for startup and UART

#define DELAYTIME 12000 // Core timer=sysclk/2=24MHz to 1KHz 
#define CS LATBbits.LATB7       // chip select pin

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable secondary osc
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // use slowest wdt?
#pragma config WINDIS = OFF // wdt no window mode
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock (24x) after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock (2x) after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

void initSPI1(void);
char SPI1_IO(unsigned char);
void setVoltage(unsigned char, unsigned char);

int main() {

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
    
    TRISBbits.TRISB4 =1;       //set pin B4 as input
    TRISAbits.TRISA4 =0;       //set pin A4 as output
    LATAbits.LATA4 = 0;        //turn on LED
    
    initSPI1();                 // Initialize SPI1
    
    __builtin_enable_interrupts();
    //LATAINV=0b10000;
  
        LATAINV=0b10000;   
    
    unsigned int sinwave[100];
    unsigned int ramp[100];
    double temp;
    int i;
    
    for (i=0;i<100;i++){
        temp=255/2+255/2*sin((2*3.14*i)/100);
        sinwave[i]=temp;                // implicit typecast to int
        temp= i*255/100;
        ramp[i]=temp;
    }
    
    
    while (1){    
        for (i=0;i<100;i++){
            while(_CP0_GET_COUNT() < 40000000/2/1000){;}
            
            setVoltage(1,sinwave[i]);
            setVoltage(0,ramp[i]);
            
            _CP0_SET_COUNT(0);
                
        }
    }
    
    return 0;
}


void initSPI1(void){
// Assign SPI1 pins
//RPB15Rbits.RPB15R=0b0010;      // Set B15 to SS1
TRISBbits.TRISB7 =0;             // Set CS to B7
CS=1;
RPA1Rbits.RPA1R=0b0011;        // Set SD01 to A1
            
// setup SPI1
  SPI1CON = 0;              // turn off the spi module and reset it
  SPI1BUF;                  // clear the rx buffer by reading from it
  SPI1BRG = 0x1000;            // baud rate to 10 MHz [SPI4BRG = (80000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0;  // clear the overflow bit
//  SPI1CONbits.MODE16=1;     // use 16 bit mode??
//  SPI1CONbits.MODE32=0;
  SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1;    // master operation
  SPI1CONbits.ON = 1;       // turn on spi 1
  
//  // send a ram set status command.
//CS = 0;                   // enable the ram
//SPI1_IO(0x01);             // ram write status
//SPI1_IO(0x41);             // sequential mode (mode = 0b01), hold disabled (hold = 0)
//CS = 1;                   // finish the command


//RPB14Rbits.RPB14R=0b;
//SDI1Rbits.SDI1R=0b0100;          // Set SDI1 to B8
}

char SPI1_IO(unsigned char write){
    SPI1BUF=write;
    while (!SPI1STATbits.SPIRBF){;} // wait to receive the byte
    return SPI1BUF;
}

void setVoltage(unsigned char channel, unsigned char voltage){
 
  // channel=0 for VoutA
  // channel=1 for VoutB
  // voltage of value 0 to 255
    
    unsigned char output[2];
    unsigned char temp,vtemp;
                     
    if (channel==1){
        temp=0b1111<<4;
    }
    else if (channel==0){
        temp=0b0111<<4;
    }
    
    vtemp=voltage>>4;
    output[1]= temp|vtemp;
    
    output[2]=voltage<<4;
    
    
//    buf[15]=channel;
//    buf[14]=1;           // Buffered OUtput
//    buf[13]=1;           // Output Gain
//    buf[12]=1;           // Output Shutdown Control bit
  
    
    CS=0;                   // Enable chip select
    SPI1_IO(output[1]);     
    SPI1_IO(output[2]);
    CS=1;                   // Disable chip select 
}

