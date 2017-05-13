#include <xc.h>           // processor SFR definitions
#include <sys/attribs.h>  // __ISR macro
#include "i2c_master_noint.h"
#include "ILI9163C.h"
#include <stdio.h>

#define SLAVE_ADDR 0b1101011
#define DELAYTIME 4800000 // Core timer=sysclk/2=24MHz to 5Hz (0.2 sec)

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

void initIMU(void);
void I2C_read_multiple(unsigned char, unsigned char *, int);
void delay(void);

unsigned char temp_data[14];

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
    
    // Turn Off Analog
    ANSELBbits.ANSB2=0;         // pin B2 (SDA2)
    ANSELBbits.ANSB3=0;         // pin B3 (SCL2)
    
    //i2c_slave_setup(SLAVE_ADDR);              // init I2C5, which we use as a slave 
                                                //  (comment out if slave is on another pic)
    i2c_master_setup();                         // init I2C2, which we use as a master
    SPI1_init();                                // Initialize SPI1
    LCD_init();

    __builtin_enable_interrupts();
    
    LATAINV=0b10000;
    
  
    
    initIMU();                      // initialize IMU
    LCD_clearScreen(BACKGROUND);    //set background color
    
    unsigned char test=0;
    i2c_master_start();                     // Begin the start sequence
    i2c_master_send(SLAVE_ADDR << 1|0);     // send the slave address, left shifted by 1 (write)
                                            // which clears bit 0, indicating a write
    i2c_master_send(0x0F);                  // read from WHO_AM_I      
    i2c_master_restart();
    
    i2c_master_send(SLAVE_ADDR<< 1|1);      // read from slave
    test=i2c_master_recv();                 
    
    i2c_master_ack(1);                      // send NACK stop sending data
    i2c_master_stop();
    
    if (test==0x69){
    
    	char msg[100];
    	sprintf(msg,"I2C working! %c",test);
    	LCD_write(msg);
    	delay();
        delay();
    }
    
    else {
    	char msg[100];
    	sprintf(msg,"I2C not working :(");
    	LCD_write(msg);
    }
    
    LCD_clearScreen(BACKGROUND);
    
    while (1){
        
        char msg[100];
//        sprintf(msg,"hello 1");
//        LCD_write(msg);
//        delay();
        
        
    // Read data from IMU
//        unsigned char temp_data[14];
        I2C_read_multiple(0x20,temp_data,14);   // start at OUT_TEMP_L register and
                                                // read through OUTZ_H_XL
        short temperature = temp_data[1] << 8 | temp_data[0];
        short gyroX = temp_data[3] << 8 | temp_data[2];
        short gyroY = temp_data[5] << 8 | temp_data[4];
        short gyroZ = temp_data[7] << 8 | temp_data[6];
        short accX = temp_data[9] << 8 | temp_data[8];
        short accY = temp_data[11] << 8 | temp_data[10];
        short accZ = temp_data[13] << 8 | temp_data[12];
        
        float accXnew=(float)accX/512;
        float accYnew=(float)accY/512;
        float accZnew=(float)accZ/512;
         
        sprintf(msg,"gyrox:%d",gyroX);
        LCD_write2(msg,32);
        sprintf(msg,"gyroy:%d",gyroY);
        LCD_write2(msg,42);
        sprintf(msg,"gyroz:%d",gyroZ);
        LCD_write2(msg,52);
        sprintf(msg,"accx:%4.2f",accXnew);
        LCD_write2(msg,62);
        sprintf(msg,"accy:%4.2f",accYnew);
        LCD_write2(msg,72);
        sprintf(msg,"accZ:%4.2f",accZnew);
        
        delay();
        
        
    
    // Draw bars on LCD    
        //unsigned char center[2]=[64,64]
//        LCD_clearScreen(BACKGROUND);
//        signed short length_x;
//        signed short length_y;
//        
//        length_x=accXnew;
//        length_y=accYnew;
//        
//        progress_bar_x(64,64,length_x);
//        progress_bar_y(64,64,length_y);
//        
//        delay();                                // delay 5Hz
//        sprintf(msg,"hello 2");
//        LCD_write(msg);
//        delay();
    }
    
    return 0;
}

void initIMU(void){
    // Turn on Accelerometer
    i2c_master_start();                     // Begin the start sequence
    i2c_master_send(SLAVE_ADDR << 1);       // send the slave address, left shifted by 1 (write)
                                            // which clears bit 0, indicating a write
    i2c_master_send(0x10);                  // write to CTRL1_XL register       
    i2c_master_send(0b10000010);            // sample rate 1.66kHz, 2g sensitivity, 100Hz filter
    i2c_master_stop();
 
    // Turn on Gyroscope
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1);       // send the slave address, left shifted by 1 (write)
    
    i2c_master_send(0x11);                  // write to CTRL2_G register       
    i2c_master_send(0b10001000);            // sample rate 1.66kHz, 1000dps sensitivity
    i2c_master_stop();
    
    // Enable multiple registers
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1);       // send the slave address, left shifted by 1 (write)
    
    i2c_master_send(0x12);                  // write to CTRL3_C register       
    i2c_master_send(0b00000100);            // set IF_INC high
    
    //i2c_master_ack(1);                      // send NACK (1):  master needs no more bytes
    i2c_master_stop();                      // send STOP:  end transmission, give up bus
}

void I2C_read_multiple(unsigned char start_register, unsigned char * data, int length){
    
    i2c_master_start();                     // Begin the start sequence
    i2c_master_send(SLAVE_ADDR << 1|0);     // send the slave address, left shifted by 1 (write)
                                            // which clears bit 0, indicating a write
    i2c_master_send(start_register);        // start read from input register      
    i2c_master_restart();
    i2c_master_send(SLAVE_ADDR<< 1|1);  // read from slave
    int i;
    for (i=0;i<(length-1);i++){
        //i2c_master_send(SLAVE_ADDR<< 1|1);  // read from slave
        data[i]=i2c_master_recv();        // store info in data array?
        //*(data+i)=i2c_master_recv();
        i2c_master_ack(0);                  // send ACK (0): master wants another byte!
    }
    data[length-1]=i2c_master_recv();
    
    i2c_master_ack(1);                      // send NACK stop sending data
    i2c_master_stop();
    
}

void delay(void){
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<DELAYTIME){;}
}
