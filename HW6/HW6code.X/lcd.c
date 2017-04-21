/* 
 * File:   lcd.c
 * Author: goldsja
 *
 * Created on April 17, 2017, 12:16 AM
 */
#include <xc.h>           // processor SFR definitions
#include <sys/attribs.h>  // __ISR macro
//#include "i2c_master_noint.h"
#include <math.h>         // math funcitons
#include <stdio.h>
#include "ILI9163C.h"

#define DELAYTIME 4800000 // Core timer=sysclk/2=24MHz to 5Hz (0.2 sec)
//#define SLAVE_ADDR 0b0100000 // A0-A2 pins are low
//#define CS LATBbits.LATB7       // chip select pin
#define BACKGROUND BLACK   // set background color
#define COUNTLENGTH 100

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

void display_character (unsigned char, unsigned char, unsigned char, unsigned char);
void progress_bar (unsigned char, unsigned char,unsigned char);
void delay(void);

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
    
    SPI1_init();                 // Initialize SPI1
    LCD_init();
    
    TRISBbits.TRISB4 =1;       //set pin B4 as input
    TRISAbits.TRISA4 =0;       //set pin A4 as output
    LATAbits.LATA4 = 0;        //turn on LED

    __builtin_enable_interrupts();
    LATAINV=0b10000;                //turn on LED to show program loaded
    
    LCD_clearScreen(BACKGROUND);    //set background color
            
    unsigned short x_val=28;
    unsigned short y_val=32;
    unsigned short color=CYAN;
    int i=0;
    int x;          //temporary x value
 
    int count;
    char msg[100];
    x=x_val;
    for (count=0;count<COUNTLENGTH+1;count++){
        
        sprintf(msg,"Hello World! %d",count);
        
        while(msg[i]){      // write msg to LCD screen
            display_character(msg[i],x,y_val,color);
            i++;
            x=x+6;          // x position shifts one character?
                            // y position stays the same
        }
        i=0;
        x=x_val;
        
        progress_bar(14,y_val+20,count);
        delay();
    }
   
    
    return 0;
}

void display_character (unsigned char c, unsigned char x, unsigned char y, unsigned char color){
    char row;
    int i,j;
    row=c-0x20;
    
    
    // check position to see if it fits?
    
    for (i=0;i<5;i++){                          // go through 5 chars of character
        if(x+j<=128){
            for (j=0; j<8;j++){                     // go through 8 bits of each char
                if ((ASCII[row][i]>>j)&1==1){       // if bit is 1
                    LCD_drawPixel(x+i,y+j,color);   // draw color 
                }
                else {                              // if bit is 0
                    LCD_drawPixel(x+i,y+j,BACKGROUND);                           
                }
            }
        }
        else if (x+j>128){
            ;
        }
    }
    
}

void progress_bar (unsigned char start_x, unsigned char start_y, unsigned char length){
        
    int i,j;
    unsigned char width=7;
    
    for (i=0;i<(length+1);i++){                                 // go through length
        for (j=0; j<(width+1);j++){                             // go through width of bar
            LCD_drawPixel(start_x+i,start_y+j,MAGENTA);     // draw color 
        }                                                
    }
    
}

void delay(void){
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<DELAYTIME){;}
}
