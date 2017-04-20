#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "i2c_master_noint.h"

//#define DELAYTIME 12000 // Core timer=sysclk/2=24MHz to 1KHz
#define SLAVE_ADDR 0b0100000 // A0-A2 pins are low

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

void initExpander(void);
void setExpander(unsigned char pin, unsigned char level);
unsigned char getExpander(void);


int main() {
    
     //Startup();

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
    
    //i2c_slave_setup(SLAVE_ADDR);              // init I2C5, which we use as a slave 
                                                 //  (comment out if slave is on another pic)
    i2c_master_setup();                              // init I2C2, which we use as a master

    __builtin_enable_interrupts();
    
    LATAINV=0b10000;
    
    // Turn Off Analog
    ANSELBbits.ANSB2=0;         // pin B2 (SDA2)
    ANSELBbits.ANSB3=0;         // pin B3 (SCL2)
    
    initExpander();
    
    unsigned char pin_val;
    
    //i2c_master_start();                     // Begin the start sequence
    
    while (1){
    
        pin_val=getExpander();                    // read value of pins
        
        if ((pin_val & 0b10000000)== 0){        // pin 7 is low (bit 7 is 0)
            setExpander(0,0);                   // set GP0 low
        }
        else if ((pin_val & 0b10000000)== 1){   // pin 7 is high (bit 7 is 1)     
            setExpander(1,0);                   // set GP0 high
        }
    
    }

    return 0;
}

void initExpander(void){
 // Set pins 4-7 as inputs and pins 0-3 as outputs
    i2c_master_start();                     // Begin the start sequence
    i2c_master_send(SLAVE_ADDR << 1);       // send the slave address, left shifted by 1, 
                                            // which clears bit 0, indicating a write
    i2c_master_send(0x00);                  // write to IODIR register       
    i2c_master_send(0b11110000);            // set pins to input/output
    
    //i2c_master_ack(1);                      // send NACK (1):  master needs no more bytes
    i2c_master_stop();                      // send STOP:  end transmission, give up bus
}

void setExpander(unsigned char level, unsigned char pin){
     // Set pins as high or lows (0 low, 1 high)
    
    int i;
    unsigned char set,temp;
    temp=1;
    for(i=0;i<8;i++){
        if (i==pin){
            set=temp<<i;
        }
    }
    
    //read current pin values
    unsigned char pin_val;
    i2c_master_start();                     // Begin the start sequence
    i2c_master_send(SLAVE_ADDR << 1);       // send the slave address, left shifted by 1, 
                                            // which clears bit 0, indicating a write
    i2c_master_send(0x09);                  // read from GPIO
    i2c_master_restart();
    i2c_master_send(SLAVE_ADDR<< 1|1);
    pin_val=i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    
    if (level==0){
        set=set^pin_val;
    }
    
    else{
        set=set|pin_val;
    }
    
    i2c_master_start();                     // Begin the start sequence
    
    i2c_master_send(SLAVE_ADDR << 1);       // send the slave address, left shifted by 1, 
                                            // which clears bit 0, indicating a write
    i2c_master_send(0x09);                  // write to GPIO register       
    i2c_master_send(set);                   // set pins to input/output
    //i2c_master_ack(1);                      // send NACK (1):  master needs no more bytes
    i2c_master_stop();                      // send STOP:  end transmission, give up bus
}

unsigned char getExpander(void){
    unsigned char pin_val;
    i2c_master_start();                     // Begin the start sequence
    i2c_master_send(SLAVE_ADDR << 1);       // send the slave address, left shifted by 1, 
                                            // which clears bit 0, indicating a write
    i2c_master_send(0x09);
    i2c_master_restart();
    i2c_master_send(SLAVE_ADDR<< 1|1);
    pin_val=i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    
    return pin_val;
    
}