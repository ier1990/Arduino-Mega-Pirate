#include <Wire.h>
#include <pins_arduino.h>

//in shiftPinNo.h
#define shPin1 1                               
#define shPin2 2                               
#define shPin3 4                               
#define shPin4 8                               
#define shPin5 16                              
#define shPin6 32                              
#define shPin7 64                              
#define shPin8 128                             
#define shPin9 256                             
#define shPin10 512 
#define shPin11 1024 
#define shPin12 2048
#define shPin13 4096
#define shPin14 8192
#define shPin15 16384
#define shPin16 32768
//32,768 is a positive integer equal to \(2^{15} = 2^{2^4 - 1}\). It is notable in computer science for being the absolute value of the maximum negative value of a 16-bit signed integer, which spans the range [-32768, 32767]

/*
 ---- Arduino Analog Logic State Analyzer v2 ----

*/
#define FASTADC 1
// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


//https://wiki.waynebooth.me/hobbie-docs/electronics/devices/microcontroller/uc-techniques
//https://www.avrfreaks.net/s/topic/a5C3l000000U7yCEAS/t044434
// from AVR035: Efficient C Coding for AVR
#ifndef _AVR035_H_
#define _AVR035_H_
#define SETBIT(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define CLEARBIT(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))
#define FLIPBIT(ADDRESS,BIT) (ADDRESS ^= (1<<BIT))
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT))

#define SETBITMASK(x,y) (x |= (y))
#define bit_set(p,m) ((p) |= (BIT(m)))

#define CLEARBITMASK(x,y) (x &= (~y))
#define bit_clear(p,m) ((p) &= ~(BIT(m)))

#define FLIPBITMASK(x,y) (x ^= (y))
#define bit_flip(p,m) ((p) ^= (BIT(m)))

#define CHECKBITMASK(x,y) (x & (y))
#define bit_get(p,m) ((p) & (BIT(m)))

#define VARFROMCOMB(x, y) x
#define BITFROMCOMB(x, y) y

#define C_SETBIT(comb) SETBIT(VARFROMCOMB(comb), BITFROMCOMB(comb))
#define C_CLEARBIT(comb) CLEARBIT(VARFROMCOMB(comb), BITFROMCOMB(comb))
#define C_FLIPBIT(comb) FLIPBIT(VARFROMCOMB(comb), BITFROMCOMB(comb))
#define C_CHECKBIT(comb) CHECKBIT(VARFROMCOMB(comb), BITFROMCOMB(comb))

#define bit_write(c,p,m) (c ? bit_set(p,BIT(m)) : bit_clear(p,BIT(m)))
#define BIT(x) (0x01 << (x))
#define LONGBIT(x) ((unsigned long)0x00000001 << (x)) 

#define LOW 0
#define HIGH 1
#define FALSE 0
#define TRUE 1
#define OFF 0
#define ON 1


#define _INPUT(port,pin) DDR ## port &= ~(1<<pin)
#define _OUTPUT(port,pin) DDR ## port |= (1<<pin)
#define CLEAR(port,pin) PORT ## port &= ~(1<<pin)
#define SET(port,pin) PORT ## port |= (1<<pin)
#define TOGGLE(port,pin) PORT ## port ^= (1<<pin)
#define READ(port,pin) (PIN ## port & (1<<pin))


//usage:Code:
//_OUTPUT(D, 3); // port D, pin 3 as output
//SET(D, 3); // set port D pin 3 to HIGH
//CLEAR(D, 3); // set it to LOW
//_INPUT(B, 5);
//if (READ(B, 5) == HIGH) 


//
// ##############

// GPIO.h file
//#define G_INPUT(port,pin) DDR ## port &= ~(1<<pin)
//#define G_OUTPUT(port,pin) DDR ## port |= (1<<pin)
//#define G_CLEAR(port,pin) PORT ## port &= ~(1<<pin)
//#define G_SET(port,pin) PORT ## port |= (1<<pin)
//#define G_TOGGLE(port,pin) PORT ## port ^= (1<<pin)
//#define G_READ(port,pin) (PIN ## port & (1<<pin))
//#define G_READ(port,pin) ((PIN ## port & (1<<pin)) >> pin)


//#define GPIO_INPUT(...)    G_INPUT(__VA_ARGS__)
//#define GPIO_OUTPUT(...)   G_OUTPUT(__VA_ARGS__)
//#define GPIO_CLEAR(...)    G_CLEAR(__VA_ARGS__)
//#define GPIO_SET(...)      G_SET(__VA_ARGS__)
//#define GPIO_TOGGLE(...)   G_TOGGLE(__VA_ARGS__)
//#define GPIO_READ(...)     G_READ(__VA_ARGS__)
//#define GPIO_READ_N(...)   G_READ_N(__VA_ARGS__)

// ########

// project.h file : keep all the definitions here

//#define LED A, 0
//#define SWITCH B, 1

// ###########

// Project.c file
// This can be used as follows

//#include "include/GPIO.h"
//#include "include/project.h"



//GPIO_SET(LED) // Turn on the LED

//if ( GPIO_READ(SWITCH)==0){ // Switch on
//}
#endif 


//Display
byte  ANALOG = ON; // ON for Arduino Auto Plotter
byte  DIGITAL = OFF;
byte  REGISTER = OFF;
//blinker
byte  globalblinkstate = OFF;


//https://a.co/d/1TUHdSQ
//Clock in data and latch it to free up IO pins on your micro.
//pins for the 74HC595
//#define strobePin  8
//#define clockPin   12
//#define dataPin10k  11
//#define enablePin10k  10
//#define dataPinvoltmod  9
//#define enablePinvoltmod  13
// pins for the other microcontroller
int strobePin = 24;
int clockPin = 22;
int dataPin10k = 30;
int enablePin10k = 32;
int dataPinvoltmod = 34;
int enablePinvoltmod = 36;


//analog pins


#define arduino1280DigitalBitMapLength 54
byte arduino1280DigitalBitMap[arduino1280DigitalBitMapLength] = {
    //[0] =WRITE PROTECT  ON/OFF or 1/0
    //[1] =digital IRQ    ON/OFF or 1/0
    //[2] =digital PWM    ON/OFF or 1/0
    //[3] =digital BLINK  ON/OFF or 1/0
    //[4] =fast           ON/OFF or 1/0 Gets more timeslices
    //[5] =digital HIGH/LOW      or 1/0
    //[6] =digital PULLUP ON/OFF or 1/0
    //[7] =digital OUTPUT/INPUT  or 1/0
            B10000000,  B10000000,  B00000000,  B00000000,  B00000000,  B00000000,  B00000000,  B00000000,  B00000000,  B0000000,//0-9 pin
            B00000000,  B00000000,  B00000000,  B00000000,  B00000000,  B00000000,  B00000000,  B00000000,  B00000000,  B0000000,//10-19 pin
            B10000000,  B10000000,  B00000001,  B00000000,  B00000001,  B00000000,  B00000001,  B00000000,  B00000001,  B0000000,//20-29 pin
            B00000001,  B00000000,  B00000001,  B00000000,  B00000001,  B00000000,  B00000001,  B00000000,  B00000001,  B0000000,//30-39 pin        
            B00000001,  B00000000,  B00000000,  B00000000,  B00000000,  B00000000,  B00000000,  B00000000,  B00000000,  B0000000,//40-49 pin
            B00000000,  B00000000,  B00000000,  B00000000//50-53 pin
};

//byte arduino1280DigitalBitMap[54] = {   
//[0] =WRITE PROTECT  ON/OFF or 1/0
//[1] =digital IRQ    ON/OFF or 1/0
//[2] =digital PWM    ON/OFF or 1/0
//[3] =digital BLINK  ON/OFF or 1/0
//[4] =fast           ON/OFF or 1/0 Gets more timeslices
//[5] =digital HIGH/LOW      or 1/0
//[6] =digital PULLUP ON/OFF or 1/0
//[7] =digital OUTPUT/INPUT  or 1/0
//int arduino1280DigitalBitMapLength=54;
//y=a given bit inside data The expression (1<<y) Then using bitwise AND, data & (1<<y) tests the given bit. 
//If that bit is 1, a nonzero value results, causing the if to see it as being true. 
//Otherwise, if the bit is 0, it is treated as false, so the else executes. 
// bit_get(p,m) ((p) & (m))
// bit_set(p,m) ((p) |= (m))
// bit_clear(p,m) ((p) &= ~(m))
// bit_flip(p,m) ((p) ^= (m))
// bit_write(c,p,m) (c ? bit_set(p,m) : bit_clear(p,m))
// BIT(x) (0x01 << (x))
// LONGBIT(x) ((unsigned long)0x00000001 << (x))              
// bit_set(arduino1280DigitalBitMap[prompt_pin_int],7); //OUTPUT
// bit_set(arduino1280DigitalBitMap[prompt_pin_int],5); //HIGH 






/*
 * Cut-and-pasted from www.arduino.cc playground section for determining heap and stack pointer locations.
 * http://www.arduino.cc/playground/Code/AvailableMemory
 *
 * Also taken from the Pololu thread from Paul at: http://forum.pololu.com/viewtopic.php?f=10&t=989&view=unread#p4218
 *
 * Reference figure of AVR memory areas .data, .bss, heap (all growing upwards), then stack growing downward:
 * http://www.nongnu.org/avr-libc/user-manual/malloc.html
 * http://www.arduino.cc/en/Tutorial/Memory
 */

extern unsigned int __data_start;
extern unsigned int __data_end;
extern unsigned int __bss_start;
extern unsigned int __bss_end;
extern unsigned int __heap_start;
//extern void *__malloc_heap_start; --> apparently already declared as char*
//extern void *__malloc_margin; --> apparently already declared as a size_t
extern void* __brkval;
// RAMEND and SP seem to be available without declaration here

int16_t ramSize = 0;   // total amount of ram available for partitioning
int16_t dataSize = 0;  // partition size for .data section
int16_t bssSize = 0;   // partition size for .bss section
int16_t heapSize = 0;  // partition size for current snapshot of the heap section
int16_t stackSize = 0; // partition size for current snapshot of the stack section
int16_t freeMem1 = 0;  // available ram calculation #1
int16_t freeMem2 = 0;  // available ram calculation #2


/* This function places the current value of the heap and stack pointers in the
 * variables. You can call it from any place in your code and save the data for
 * outputting or displaying later. This allows you to check at different parts of
 * your program flow.
 * The stack pointer starts at the top of RAM and grows downwards. The heap pointer
 * starts just above the static variables etc. and grows upwards. SP should always
 * be larger than HP or you'll be in big trouble! The smaller the gap, the more
 * careful you need to be. Julian Gall 6-Feb-2009.
 */
uint8_t* heapptr, * stackptr;
uint16_t diff = 0;
void check_mem() {
    stackptr = (uint8_t*)malloc(4);          // use stackptr temporarily
    heapptr = stackptr;                     // save value of heap pointer
    free(stackptr);      // free up the memory again (sets stackptr to 0)
    stackptr = (uint8_t*)(SP);           // save value of stack pointer
}


/* Stack and heap memory collision detector from: http://forum.pololu.com/viewtopic.php?f=10&t=989&view=unread#p4218
 * (found this link and good discussion from: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1213583720%3Bstart=all )
 * The idea is that you need to subtract your current stack pointer (conveniently given by the address of a local variable)
 * from a pointer to the top of the static variable memory (__bss_end). If malloc() is being used, the top of the heap
 * (__brkval) needs to be used instead. In a simple test, this function seemed to do the job, showing memory gradually
 * being used up until, with around 29 bytes free, the program started behaving erratically.
 */
 //extern int __bss_end;
 //extern void *__brkval;

int get_free_memory()
{
    int free_memory;

    if ((int)__brkval == 0)
        free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
        free_memory = ((int)&free_memory) - ((int)__brkval);

    return free_memory;
}

void display_free_memory()
{
    Serial.println("\n--- Free Memory in bytes---");
    Serial.print("Free Memory=");
    Serial.println(get_free_memory());
    check_mem();
    diff = stackptr - heapptr;
    Serial.print("Stack-Heap=");
    Serial.println((int)diff, DEC);
    // summaries:
    ramSize = (int)RAMEND - (int)&__data_start;
    dataSize = (int)&__data_end - (int)&__data_start;
    bssSize = (int)&__bss_end - (int)&__bss_start;
    heapSize = (int)__brkval - (int)&__heap_start;
    stackSize = (int)RAMEND - (int)SP;
    freeMem1 = (int)SP - (int)__brkval;
    freeMem2 = ramSize - stackSize - heapSize - bssSize - dataSize;
    Serial.println("--- section size summaries in bytes---");
    Serial.print("ram   size="); Serial.println(ramSize, DEC);
    Serial.print(".data size="); Serial.println(dataSize, DEC);
    Serial.print(".bss  size="); Serial.println(bssSize, DEC);
    Serial.print("heap  size="); Serial.println(heapSize, DEC);
    Serial.print("stack size="); Serial.println(stackSize, DEC);
    Serial.print("free size1="); Serial.println(freeMem1, DEC);
    Serial.print("free size2="); Serial.println(freeMem2, DEC);

}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void PausePressKey()
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PausePressKey(int pause = true)
{
    if (!pause) { return; }
    print_milli_timer();
    Serial.println();
    Serial.print("Pause press a key->_");
    while (Serial.read() == -1);
    print_milli_timer();
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void displayhelp()
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void displayhelp(int pause = true)
{
    Serial.println();
    Serial.println("Arduino Logic State Analyzer");
    Serial.println("Press ?  for Available Commands");
    Serial.println("Press /  to create Commandline");
    //Serial.println();
    Serial.println("-----------------Running Commandline--------------------");
    Serial.println("Press g  Faster Scan A=/.3v  resolution");
    Serial.println("Press f  Fast Scan   A=.02v  resolution");
    Serial.println("Press s  Slow Scan   A=.005v resolution");
    Serial.println("Press t  Displays miliseconds since program start");
    Serial.println("Press a  Analog Scan");
    Serial.println("Press d  Digital Scan");
    Serial.println("Press m  Available Memory");
    Serial.println("Press i  I2c Scanner & arduino1280DigitalBitMap Array");
    Serial.println("Press z  Register Port F&K Scan");
    Serial.println("Press y  Register Port E&H Scan");
    Serial.println("Press x  ALL Port Port A-L Scan");
    Serial.println("Press e  8 Bit Shift Register Pins:clock" + String(clockPin) + ",enable:" + String(enablePin10k) + ",data:" + String(dataPin10k));
    Serial.println("Press w  8 Bit Shift Register Pins:clock" + String(clockPin) + ",enable:" + String(enablePinvoltmod) + ",data:" + String(dataPin10k));
    Serial.println("Press p  Prints all digitalPinToBitMask to digitalPinToPort " + String(arduino1280DigitalBitMapLength) + " Digital Pins");
    PausePressKey(pause);
    //Serial.println();
    Serial.println("-----------------/W WRITE------------------------------");
    Serial.println("wb#     Makes pin# Output and BLINKS it HIGH/LOW");
    //WriteBlink runs in main loop
    Serial.println("df/ds#  BLINKS ~8khz Runs 1 sec then pauses for 100 millisecs for Serial.read");
    //DigitalWrite     = /ds# = 1366us
    //DigitalWriteFast = /df# = 62us !!!!!!!!!!!!!!!!!!!!!!!MUCH BETTER
    //No Serial.Read or TX for 1 sec while digitalWriteFast 1/0 pin with same timing loop takes 62us per loop 
    Serial.println("wh#     Makes pin# Output as HIGH ");
    Serial.println("wu#     Makes pin# PullUp ");
    Serial.println("wl#     Makes pin# Output as LOW");
    Serial.println("wi#     Makes pin# as Input");
    //Serial.println();
    Serial.println("-----------------/R READ------------------------------");
    Serial.println("rx      1sec TX 115kbaud(12kchars48kbitstatesMax) using HEX Protacol");
    //One Char takes 1/12000 = 80us freq so 12k is the freq aka samples per second not complete cycles !
    //Serial.println("Samples 1 sec then pauses for 100 millisecs for Serial.read");
    Serial.println("rd#     Read digital pin# value digitalRead(");
    //DigitalRead      = /rd# = 80us  NoSerial.print=(14us ~ 71k reads per sec)       
    Serial.println("re#     Read digital pin# value digitalReadFast(");
    //DigitalReadFast  = /re# = 80us  NoSerial.print=( 8us ~125k reads per sec)
    Serial.println("ra#     Read analog  pin# value A=.3v  resolution");
    //analogRead() with No Serial.Read or TX for 1 sec  8300 count at 120us per read      
    //analogRead() with    Serial.Read or TX for 1 sec  6914 count at 144us per read
    //With FASTADC with    Serial.Read or TX for 1 sec 11768 count at  84us per read  !!!!!!!!!!!!!!!!!!!!!!!MUCH BETTER
    //With FASTADC with No Serial.Read or TX for 1 sec 36948 count at  27us per read  
    Serial.println("-----------------/ Expermental or Broke-----------------");
    Serial.println("rf#     Measures Freq pin# Reads");
    Serial.println("b$      $ = letter of register port to read a,b,c,d,e,f,g,h,j,k,l");
    Serial.println();
    Serial.println("rc#     Read Capacitance value analog pin# 0-15, takes ~ 2secs ");
    // add in any extra help here
    PausePressKey(pause);
}

//Speed info added here from all test at 115kbaud 12k chars per sec 48k bit states Max at 80us just with  Serial.println("|"); and loop
//everything else add to that time 80us and it can change at +-10% or worse depending on serial speeds
//analogRead       = /ra# ~ 81us to 122us.
//analogReadFast   = Not created yet!
//*****************************************************************************//*****************************************************************************//*****************************************************************************
//Note: Running  a g = ~2900us so (22chars*80us=2320us) - (16analogReads*80us=1280us)  = 1040us of wasted time?
//**********************Current Views******************************************//*****************************************************************************//*****************************************************************************
//analogReadFast  only reads 6kchars per sec 57kbaud still the 161us total.     
//DigitalRead     reads       12000 samples per second ~ 80us so it's fine for now untill we speed up analog & Serial Lib.      
//analogRead      is         1023/68=0to16 or Hex 0toF = .3volt resolution for char TX using HEX Protacol
//analogReadFast needs to be ??us before HEX Protacol is dropped for something with better speed
//analogRead (161us-80uscharTX_MAX) = 81us for a single analog read 
//1 char at 115kbaud takes ~ 80us so 16analogReads*80us=1280us of time to create
//Serial.print + Serial.Read = ~22us
//No Serial.Read or TX for 1 sec while digitalWriteFast 1/0 pin with same timing loop takes 62us 
//so 84-62=22us so 22us went to SerialRead & SerialWrite Lib still faster then analogRead at 84us??
// = ~84usto120us to analogRead a single pin










//speed
#define  FASTER 3
#define  FAST   2
#define  SLOW   1 // for Arduino Auto Plotter
byte speeder = SLOW; // for Arduino Auto Plotter
unsigned long time;
unsigned long old_time;
long interval = 1000000L;//1sec





//Create an integer arrays for data from the Analog Input
#define analog_pins_min 0
#define analog_pins_max 16
#define analog_safety_pins_min 0
#define analog_safety_pins_max 16
int analogarray[analog_pins_max];


//Create an integer arrays for data from the Digital Input
#define digital_pins_min 0
#define digital_pins_max 54
#define digital_safety_pins_min 2
#define digital_safety_pins_max 53
int digitalarray[digital_pins_max];


//Create an btye arrays for data from the serial in
//byte  commandline_max_len = 80;
#define commandline_max_len 80
byte commandline[80];
//byte LineOfText1[80];
unsigned long characterToStringPointer = 0;

#define resistorValueLI  10000.0F   // change this to whatever resistor value you are using
#define resistorValueHI    180.0F   // change this to whatever resistor value you are using
// F formatter tells compliler it's a floating point value

uint8_t status;                   /* Define a 8 bit integer variable */
char registerw;

int receivedchar; //var that will hold the bytes in read from the serialBuffer


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Start Setup
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
    Wire.begin();

#if FASTADC
    // set prescale to 16
    sbi(ADCSRA, ADPS2);
    cbi(ADCSRA, ADPS1);
    cbi(ADCSRA, ADPS0);
#endif  

    //  Initiate Serial Communication
    //  Serial.begin(57600);
    Serial.begin(115200);
    ResetCommandLine();
    for (int x = 0; x < arduino1280DigitalBitMapLength; ++x)
    {
        Serial.println();
        Serial.print("pin=");
        Serial.print(x);
        Serial.print("|");
        Serial.print("digitalPinToBitMask(x)=");
        printBits(digitalPinToBitMask(x));

        Serial.print("|PORT");
        Serial.print((digitalPinToPort(x) + 64));
        Serial.print(" digitalPinToPort(x)=");
        Serial.print("=");
        printBits(digitalPinToPort(x));


    }
    //PausePressKey();
    Serial.println();
    display_free_memory();
    displayhelp(false);


    pinMode(strobePin, OUTPUT);
    pinMode(clockPin, OUTPUT);
    pinMode(dataPin10k, OUTPUT);
    pinMode(enablePin10k, OUTPUT);
    pinMode(dataPinvoltmod, OUTPUT);
    pinMode(enablePinvoltmod, OUTPUT);

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// End Setup
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void display_arduino1280ArrayLength()
{
    Serial.println();
    Serial.println("arduino1280DigitalBitMap: ");
    for (int x = 0; x < arduino1280DigitalBitMapLength; ++x)
    {
        Serial.print(x);
        Serial.print("=");
        printBits(arduino1280DigitalBitMap[x]);
        Serial.print(",");
    }
    Serial.println();
    Serial.print("globalblinkstate=");
    Serial.println(globalblinkstate, DEC);
    PausePressKey();
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Start Main Program loop() runned forever
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{

    //gets time  
    old_time = time;
    time = millis();

    if (time - old_time > interval)
    {
        old_time = time;
    }

    int nDevices = 0;


    if (globalblinkstate == ON)//blink
    {
        for (int i = 0; i < arduino1280DigitalBitMapLength; i++)
        {
            byte data = arduino1280DigitalBitMap[i];   // fetch data from memory
            if ((!bit_get(data, 0)) && (bit_get(data, 3)) && (bit_get(data, 7)))
            {
                bit_flip(arduino1280DigitalBitMap[i], 5);
                digitalWrite(i, bit_get(arduino1280DigitalBitMap[i], 5));
            }
        }

    }


    ///////////////////////////////////////////
    //Start of Transmission  
    ///////////////////////////////////////////
    display_pinstates();


    ////////////////////////////////////////////
    //Start of Receive //Start of Receive 
    ////////////////////////////////////////////
    if (Serial.available())
    {
        receivedchar = Serial.read(); //read Serial
        //receivedchar = receivedchar.toLowerCase();
    }

    //  'If we Received anything do something with it
    if (receivedchar > -1)
    {
        if ((receivedchar == '/') || (commandline[0] == '/'))
        {
            CreateCommandLine(receivedchar);
        }
        else if (receivedchar == 's') {
            speeder = SLOW;
        }
        else if (receivedchar == 'f') {
            speeder = FAST;
        }
        else if (receivedchar == 'g') {
            speeder = FASTER;
        }
        else if (receivedchar == 'a') {
            if (ANALOG == ON) { ANALOG = OFF; }
            else { ANALOG = ON; }
        }
        else if (receivedchar == 'd') {
            if (DIGITAL == ON) { DIGITAL = OFF; }
            else { DIGITAL = ON; }
        }
        else if (receivedchar == 'i') {
            display_arduino1280ArrayLength();

            //  Scan the I2C bus for devices
        // --------------------------------------
        // i2c_scanner
        // Version 1
        //    This program (or code that looks like it)
        //    can be found in many places.
        //    For example on the Arduino.cc forum.
        //    The original author is not known.//
        // This sketch tests the standard 7-bit addresses
        // Devices with higher bit address might not be seen properly.
        //
            for (byte address = 1; address < 127; ++address) {
                // The i2c_scanner uses the return value of
                // the Wire.endTransmission to see if
                // a device did acknowledge to the address.
                Wire.beginTransmission(address);
                byte error = Wire.endTransmission();

                if (error == 0) {
                    Serial.print("I2C device found at address 0x");
                    if (address < 16) {
                        Serial.print("0");
                    }
                    Serial.print(address, HEX);
                    Serial.println("  !");

                    ++nDevices;
                }
                else if (error == 4) {
                    Serial.print("Unknown error at address 0x");
                    if (address < 16) {
                        Serial.print("0");
                    }
                    Serial.println(address, HEX);
                }
            }
            if (nDevices == 0) {
                Serial.println("No I2C devices found\n");
            }
            else {
                Serial.println("done");
            }
            PausePressKey();
        }

        else if (receivedchar == 'z') {
            if (REGISTER == ON) { REGISTER = OFF; }
            else { REGISTER = ON; }
            registerw = 'z';
        }
        else if (receivedchar == 'x') {
            if (REGISTER == ON) { REGISTER = OFF; }
            else { REGISTER = ON; }
            registerw = 'x';
        }

        else if (receivedchar == 'y') {
            if (REGISTER == ON) { REGISTER = OFF; }
            else { REGISTER = ON; }
            registerw = 'y';
        }

        else if (receivedchar == 'm') {
            Serial.println();
            display_free_memory();
            PausePressKey();
        }

        else if (receivedchar == 'w') {

            for (int j = 0; j < 2; j++) {
                //ground latchPin and hold low for as long as you are transmitting
                digitalWrite(strobePin, LOW);
                digitalWrite(enablePin10k, HIGH);
                shiftOut(dataPin10k, clockPin, LSBFIRST, j);
                //return the latch pin high to signal chip that it
                //no longer needs to listen for information
                digitalWrite(strobePin, HIGH);
                //    digitalWrite(enablePin10k, LOW);   
                //    delay(1000);
            }
            //                          PausePressKey();
        }

        else if (receivedchar == 'e') {
            //https://www.arduino.cc/reference/en/language/functions/advanced-io/shiftout/
            //ground latchPin and hold low for as long as you are transmitting
            digitalWrite(strobePin, LOW);
            digitalWrite(enablePinvoltmod, HIGH);
            shiftOut(dataPinvoltmod, clockPin, LSBFIRST, 0);
            shiftOut(dataPinvoltmod, clockPin, LSBFIRST, 1);
            shiftOut(dataPinvoltmod, clockPin, LSBFIRST, 0);
            shiftOut(dataPinvoltmod, clockPin, LSBFIRST, 0);
            //return the latch pin high to signal chip that it
            //no longer needs to listen for information
            digitalWrite(strobePin, HIGH);
            //digitalWrite(enablePinvoltmod, LOW);   


        }
        else if (receivedchar == 'p') {
            for (int x = 0; x < arduino1280DigitalBitMapLength; ++x)
            {
                Serial.println();
                Serial.print("pin=");
                Serial.print(x);
                Serial.print("|");
                Serial.print("digitalPinToBitMask(x)=");
                printBits(digitalPinToBitMask(x));

                Serial.print("|PORT");
                Serial.print((digitalPinToPort(x) + 64));
                Serial.print(" digitalPinToPort(x)=");
                Serial.print("=");
                printBits(digitalPinToPort(x));


            }
            PausePressKey();
        }


        else if (receivedchar == 't') {
            print_milli_timer();
        }
        else if (receivedchar == '?') {
            displayhelp(true);
        }
        else
        {

            //-----------------------------------------------------------------------------  
            // Command Prompt
            // No constant TX
            // All commands still work
            // Anything printed here will NOT print over & over but will echo it to screen once
            // TODO:
            //   Add a one time C:\ that displays 
            //-----------------------------------------------------------------------------

            if (receivedchar == 13)
            {
                Serial.println();
                Serial.print("?");
            }
            //Serial.print("ASCII=");  
            Serial.print(receivedchar);
            //print_milli_timer();
        }
    }

    receivedchar = -1; // erase old receive
    ////////////////////////////////////////////
    ////////////////////////////////////////////
    //  End of Receive   End of Receive End of Receive 
    ////////////////////////////////////////////
    ////////////////////////////////////////////



    ///////////////////////////////////////////  
    //End of Transmission and main loop()     
    ///////////////////////////////////////////

    if (
        //  (characterToStringPointer > 0)
        //  &&
        (ANALOG != ON)
        &&
        (DIGITAL != ON)
        &&
        (REGISTER != ON)
        )
    {


        //-----------------------------------------------------------------------------  
        // Command Prompt
        // No constant TX
        // All commands still work
        // Anything printed here will print over & over since we are in the main loop
        // TODO:
        //   Add a one time C:\ that displays 
        //-----------------------------------------------------------------------------


    }
    else
    {

        ///////////////////////////////////////////  
        // Displays commandline if present at end of Transmission, 
        // this also displays what is being typed for receiving software or human,
        // basically it is the command prompt. Pressing ENTER sends prompt_string to ProcessCommandLine
        ///////////////////////////////////////////  
        if (characterToStringPointer > 0)
        {
            Serial.print(" [");
            for (int i = 0; i < characterToStringPointer; i++)
            {
                //Serial.print(commandline[i]);//prints the character just read 
                Serial.print((char)commandline[i]);
            }
            Serial.print("] ");
            Serial.print(characterToStringPointer);
        }


        display_timestates();
        Serial.println();
    }
    ///////////////////////////////////////////  
    //End of Transmission and main loop()
    ///////////////////////////////////////////
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// End Main Program loop() runned forever
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// void display_timestates()
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void display_timestates()
{
    if (speeder == SLOW)
    {
        //Transmit of time 
        Serial.print('T');
        Serial.print(time - old_time);
        Serial.print("ms");
    }
    else
    {
        //Transmit of time   
        Serial.print('T');
        Serial.print(time - old_time);
        Serial.print("ms");
    }

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// End void display_timestates()
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void display_pinstates()
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void display_pinstates()
{
    if (ANALOG == ON)
    {
        //read_all_analogpins();
        //Serial.print('A ');
        //      Serial.print(speeder);
        Serial.print('[');
        draw_analog(speeder);
        Serial.print(']');
    }
    if (DIGITAL == ON)
    {
        read_all_digitalpins();
        //Serial.print('D ');
        //      Serial.print(speeder);
        Serial.print('[');
        draw_digital(speeder);
        Serial.print(']');
    }
    if (REGISTER == ON)
    {
        if (registerw != 'z' || registerw != 'x' || registerw != 'y') { Serial.print(registerw); }
        switch (registerw)
        {
        case 'a':
            status = PINA;                   /* Store the current value of PIND in status */
            printBits(status);
            break;
        case 'b':
            status = PINB;                   /* Store the current value of PIND in status */
            printBits(status);
            break;
        case 'c':
            status = PINC;                   /* Store the current value of PIND in status */
            printBits(status);
            break;
        case 'd':
            status = PIND;                   /* Store the current value of PIND in status */
            printBits(status);
            break;
        case 'e':
            status = PINE;                   /* Store the current value of PIND in status */
            printBits(status);
            break;
        case 'f':
            status = PINF;                   /* Store the current value of PIND in status */
            printBits(status);
            break;
        case 'g':
            status = PING;                   /* Store the current value of PIND in status */
            printBits(status);
            break;
        case 'h':
            status = PINH;                   /* Store the current value of PIND in status */
            printBits(status);
            break;
        case 'j':
            status = PINJ;                   /* Store the current value of PIND in status */
            printBits(status);
            break;
        case 'k':
            status = PINK;                   /* Store the current value of PIND in status */
            printBits(status);
            break;
        case 'l':
            status = PINL;                   /* Store the current value of PIND in status */
            printBits(status);
            break;

        case 'x':
            if (speeder == SLOW)
            {
                Serial.print('a');
                status = PINA;                   /* Store the current value of PIND in status */
                printBits(status);
                Serial.print('|');
                Serial.print('b');
                status = PINB;                   /* Store the current value of PIND in status */
                printBits(status);
                Serial.print('|');
                Serial.print('c');
                status = PINC;                   /* Store the current value of PIND in status */
                printBits(status);
                Serial.print('|');
                Serial.print('d');
                status = PIND;                   /* Store the current value of PIND in status */
                printBits(status);
                Serial.print('|');
                Serial.print('e');
                status = PINE;                   /* Store the current value of PIND in status */
                printBits(status);
                Serial.print('|');
                Serial.print('f');
                status = PINF;                   /* Store the current value of PIND in status */
                printBits(status);
                Serial.print('|');
                Serial.print('g');
                status = PING;                   /* Store the current value of PIND in status */
                printBits(status);
                Serial.print('|');
                Serial.print('h');
                status = PINH;                   /* Store the current value of PIND in status */
                printBits(status);
                Serial.print('|');
                Serial.print('j');
                status = PINJ;                   /* Store the current value of PIND in status */
                printBits(status);
                Serial.print('|');
                Serial.print('k');
                status = PINK;                   /* Store the current value of PIND in status */
                printBits(status);
                Serial.print('|');
                Serial.print('l');
                status = PINL;                   /* Store the current value of PIND in status */
                printBits(status);

            }
            else
            {
                Serial.print("$");
                Serial.print('a');
                status = PINA;                   /* Store the current value of PIND in status */
                Serial.print(status, HEX);
                //Serial.print('|');
                Serial.print('b');
                status = PINB;                   /* Store the current value of PIND in status */
                Serial.print(status, HEX);
                //Serial.print('|');
                Serial.print('c');
                status = PINC;                   /* Store the current value of PIND in status */
                Serial.print(status, HEX);
                //Serial.print('|');
                Serial.print('d');
                status = PIND;                   /* Store the current value of PIND in status */
                Serial.print(status, HEX);
                //Serial.print('|');
                Serial.print('e');
                status = PINE;                   /* Store the current value of PIND in status */
                Serial.print(status, HEX);
                //Serial.print('|');
                Serial.print('f');
                status = PINF;                   /* Store the current value of PIND in status */
                Serial.print(status, HEX);
                //Serial.print('|');
                Serial.print('g');
                status = PING;                   /* Store the current value of PIND in status */
                Serial.print(status, HEX);
                //Serial.print('|');
                Serial.print('h');
                status = PINH;                   /* Store the current value of PIND in status */
                Serial.print(status, HEX);
                //Serial.print('|');
                Serial.print('j');
                status = PINJ;                   /* Store the current value of PIND in status */
                Serial.print(status, HEX);
                //Serial.print('|');
                Serial.print('k');
                status = PINK;                   /* Store the current value of PIND in status */
                Serial.print(status, HEX);
                //Serial.print('|');
                Serial.print('l');
                status = PINL;                   /* Store the current value of PIND in status */
                Serial.print(status, HEX);
            }
            break;


        case 'z':
            if (speeder == SLOW)
            {
                Serial.print('f');
                status = PINF;                   /* Store the current value of PIND in status */
                printBits(status);
                Serial.print('k');
                status = PINK;                   /* Store the current value of PIND in status */
                printBits(status);
            }
            else
            {
                Serial.print("$");
                status = PINF;                   /* Store the current value of PIND in status */
                Serial.print(status, HEX);
                Serial.print('|');
                status = PINK;                   /* Store the current value of PIND in status */
                Serial.print(status, HEX);
            }
            break;


        case 'y':
            if (speeder == SLOW)
            {
                Serial.print("DDRe");
                status = DDRE;                   /* Store the current value of PIND in status */
                printBits(status);
                Serial.print("DDRh");
                status = DDRH;                   /* Store the current value of PIND in status */
                printBits(status);


                Serial.print("|PORTe");
                status = PORTE;                   /* Store the current value of PIND in status */
                printBits(status);
                Serial.print("PORT=h");
                status = PORTH;                   /* Store the current value of PIND in status */
                printBits(status);


            }
            else
            {
                Serial.print("$");
                status = DDRE;                   /* Store the current value of PIND in status */
                Serial.print(status, HEX);
                Serial.print('|');
                status = DDRH;                   /* Store the current value of PIND in status */
                Serial.print(status, HEX);
            }
            break;




        default:
            Serial.println("How the Hell did I end up here?");
            // if nothing else matches, do the default
            // default is optional
        }

    }

}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// int ProcessCommandLine()
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int ProcessCommandLine()
{
    debug_commandline(FALSE);// for debug

    char promptchar0;
    char promptchar1;
    char promptchar2;

    int prompt_pin_int = FALSE;


    promptchar0 = commandline[0]; //the '/'    command
    if (promptchar0 == '/')
    {
        ///////////////////////////////////////////////////////////////
        // Start of Control Chars digits 1,2
        ///////////////////////////////////////////////////////////////
        if (is_ASCII_letter(commandline[1])) { promptchar1 = commandline[1]; } //the first  command
        if (is_ASCII_letter(commandline[2])) { promptchar2 = commandline[2]; } //the second command
        ///////////////////////////////////////////////////////////////
        // END of Control Chars digits 1,2
        ///////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////
        // Checks to see if a number & returns it if it does ELSE return -9999
        ///////////////////////////////////////////////////////////////
        prompt_pin_int = create_ascii_number(commandline[3], commandline[4], commandline[5]);
        ///////////////////////////////////////////////////////////////
        // END of number digits 3,4,5
        ///////////////////////////////////////////////////////////////

        //echo command
        debug_commandline(FALSE);

        ////////////////////////////////////////////////////////////////  
        //Start to process command and pin number
        ////////////////////////////////////////////////////////////////


        ////////////////////////////////////////////////////////////////  
        //READ
        ////////////////////////////////////////////////////////////////
        if (promptchar1 == 'b')
        {
            registerw = promptchar2;//no error checking to see if register exist
            REGISTER = ON;
            return 1;
        }

        if (promptchar1 == 'r')
        {
            long tmpcount = 0;
            switch (promptchar2)
            {
            case 'd':
                while (Serial.read() == -1)
                {
                    time = micros();
                    tmpcount++;
                    //if(!isdigitalpinvalid(prompt_pin_int)){return -9999;}                
                    Serial.print(digitalRead(prompt_pin_int), DEC); // Send the pin value
                    //digitalRead(prompt_pin_int); // Send the pin value                
                    if ((tmpcount % 79) == 0)  // x now contains 0 ~ 80chars per line transmit
                    {
                        Serial.println();
                    }
                    if (time - old_time > interval)
                    {
                        Serial.println();
                        Serial.print("pin=");
                        Serial.print(prompt_pin_int);
                        Serial.print(" ");
                        Serial.print("count=");
                        Serial.print(tmpcount);
                        Serial.print(" in ");
                        Serial.print(time - old_time);
                        Serial.print("us");
                        Serial.print(" avg=");
                        Serial.print(((time - old_time) / tmpcount));
                        Serial.print("us");
                        Serial.println();
                        delay(100);
                        tmpcount = 0;
                        time = micros();
                        old_time = time;
                    }

                }
                return 1;
                break;



            case 'e':
                while (Serial.read() == -1)
                {
                    time = micros();
                    tmpcount++;
                    //if(!isdigitalpinvalid(prompt_pin_int)){return -9999;}                
                    Serial.print(digitalReadFast(prompt_pin_int), DEC); // Send the pin value
                    //digitalReadFast(prompt_pin_int); // Send the pin value                
                    if ((tmpcount % 79) == 0)  // x now contains 0 ~ 80chars per line transmit
                    {
                        Serial.println();
                    }
                    if (time - old_time > interval)
                    {
                        Serial.println();
                        Serial.print("pin=");
                        Serial.print(prompt_pin_int);
                        Serial.print(" ");
                        Serial.print("count=");
                        Serial.print(tmpcount);
                        Serial.print(" in ");
                        Serial.print(time - old_time);
                        Serial.print("us");
                        Serial.print(" avg=");
                        Serial.print(((time - old_time) / tmpcount));
                        Serial.print("us");
                        Serial.println();
                        delay(100);
                        tmpcount = 0;
                        time = micros();
                        old_time = time;
                    }

                }
                return 1;
                break;





            case 'a':

                while (Serial.read() == -1)
                {
                    time = micros();
                    tmpcount++;
                    //if(!isanalogpinvalid(prompt_pin_int)){return -9999;}
                    //if(isdigitalpinvalid(prompt_pin_int)){return -9999;}                

                    //Serial.print( (analogRead(prompt_pin_int)/4) ,HEX); // Send the pin value
                    //printHex1((analogRead(prompt_pin_int) / 68));
                    Serial.print(voltage(analogRead(prompt_pin_int)), 2);
                    Serial.print(", ");
                    //analogRead(prompt_pin_int);
                    //if( (tmpcount % 79) == 0)  // x now contains 0
                    //  {Serial.println();}
                    //else
                    //  {Serial.print(",");}

                    if (time - old_time > interval)
                    {
                        Serial.println();
                        Serial.print("pin=");
                        Serial.print(prompt_pin_int);
                        Serial.print(" ");
                        Serial.print("count=");
                        Serial.print(tmpcount);
                        Serial.print(" in ");
                        Serial.print(time - old_time);
                        Serial.print("us");
                        Serial.print(" avg=");
                        Serial.print(((time - old_time) / tmpcount));
                        Serial.print("us");
                        Serial.println();
                        delay(100);
                        tmpcount = 0;
                        time = micros();
                        old_time = time;
                    }

                }
                return 1;
                break;


            case 'x':

                while (Serial.read() == -1)
                {
                    time = micros();
                    tmpcount++;

                    Serial.print('|');
                    //if( (tmpcount % 79) == 0)  // x now contains 0
                    //  {Serial.println();}
                    //else
                    //  {Serial.print(",");}

                    if (time - old_time > interval)
                    {
                        Serial.println();
                        Serial.print("Chars TX=");
                        Serial.print(tmpcount);
                        Serial.print(" in ");
                        Serial.print(time - old_time);
                        Serial.print("us");
                        Serial.print(" avg per char=");
                        Serial.print(((time - old_time) / tmpcount));
                        Serial.print("us");


                        Serial.println();
                        delay(100);
                        tmpcount = 0;
                        time = micros();
                        old_time = time;
                    }

                }
                return 1;
                break;


            case 's':

                while (Serial.read() == -1)
                {
                    time = micros();
                    tmpcount++;
                    //if(!isanalogpinvalid(prompt_pin_int)){return -9999;}
                    //if(isdigitalpinvalid(prompt_pin_int)){return -9999;}                

                    //Serial.print( (analogRead(prompt_pin_int)/4) ,HEX); // Send the pin value
                    printHex1((analogRead(prompt_pin_int) / 68));

                    //Serial.print( (analogRead(prompt_pin_int)/4) ,HEX); // Send the pin value
                    printHex1((analogRead(prompt_pin_int + 1) / 68));


                    if ((tmpcount % 79) == 0)  // x now contains 0
                    {
                        Serial.println();
                    }
                    //else
                    //  {Serial.print(",");}

                    if (time - old_time > interval)
                    {
                        Serial.println();
                        Serial.print("pin=");
                        Serial.print(prompt_pin_int);
                        Serial.print(" ");
                        Serial.print("count=");
                        Serial.print(tmpcount);
                        Serial.print(" in ");
                        Serial.print(time - old_time);
                        Serial.print("us");
                        Serial.print(" avg=");
                        Serial.print(((time - old_time) / tmpcount));
                        Serial.print("us");
                        Serial.println();
                        delay(100);
                        tmpcount = 0;
                        time = micros();
                        old_time = time;
                    }

                }
                return 1;
                break;

            case 'p':
                Serial.println("digitalPinToBitMask");
                printBits(digitalPinToBitMask(prompt_pin_int));
                Serial.print("digitalPinToPort");
                printBits(digitalPinToPort(prompt_pin_int));

                break;
       
            case 'f':
                //                if(isanalogpinvalid(prompt_pin_int)){return -9999;}
                //                if(!isdigitalpinvalid(prompt_pin_int)){return -9999;}                

                display_freq(prompt_pin_int);
                break;

            case 'c':
                //                if(isanalogpinvalid(prompt_pin_int)){return -9999;}
                //                if(!isdigitalpinvalid(prompt_pin_int)){return -9999;}                

                //display_capacitance(prompt_pin_int);
                //void read_cap(int chargePin, int analogPin, int resistorValue)
                read_cap(13, prompt_pin_int, 10000);
                break;


            default:
                Serial.print("What the Hell does this mean?");  // for debug
                // if nothing else matches, do the default
                // default is optional
            }

        }

        ////////////////////////////////////////////////////////////////  
        //Write
        ////////////////////////////////////////////////////////////////
        if (promptchar1 == 'w')
        {
            switch (promptchar2) {
            case 'h':
                if (isdigitalpinvalid(prompt_pin_int))
                {
                    bit_set(arduino1280DigitalBitMap[prompt_pin_int], 7);
                    bit_set(arduino1280DigitalBitMap[prompt_pin_int], 5);
                    digital_pin_change(prompt_pin_int, HIGH, OUTPUT);
                }
                break;

            case 'l':
                if (isdigitalpinvalid(prompt_pin_int))
                {
                    bit_clear(arduino1280DigitalBitMap[prompt_pin_int], 5); //LOW              
                    bit_set(arduino1280DigitalBitMap[prompt_pin_int], 7); //OUTPUT            
                    digital_pin_change(prompt_pin_int, LOW, OUTPUT);
                }
                break;

            case 'u':
                if (isdigitalpinvalid(prompt_pin_int))
                {
                    bit_clear(arduino1280DigitalBitMap[prompt_pin_int], 5); //LOW              
                    bit_set(arduino1280DigitalBitMap[prompt_pin_int], 7); //OUTPUT            
                    digital_pullup(prompt_pin_int);
                }
                break;

            case 'b':
                if (isdigitalpinvalid(prompt_pin_int))
                {
                    blinker(prompt_pin_int);
                }
                break;

            case 'i':
                if (isdigitalpinvalid(prompt_pin_int))
                {
                    bit_clear(arduino1280DigitalBitMap[prompt_pin_int], 7); //INPUT            
                    digital_pin_change(prompt_pin_int, LOW, INPUT);
                }

                break;


            default:
                Serial.print("What the Hell does this mean?");// for debug           
                // if nothing else matches, do the default
                // default is optional
            }
        }
        ////////////////////////////////////////////////////////////////  
        //Write FAST
        ////////////////////////////////////////////////////////////////

        if (promptchar1 == 'd')
        {
            int tmpcount = 0;
            switch (promptchar2) {
            case 'f':
                pinMode(prompt_pin_int, OUTPUT);

                while (Serial.read() == -1)
                {
                    time = micros();
                    tmpcount++;
                    //if(!isdigitalpinvalid(prompt_pin_int)){return -9999;}
                    bit_flip(arduino1280DigitalBitMap[prompt_pin_int], 5);
                    digitalWriteFast(prompt_pin_int, bit_get(arduino1280DigitalBitMap[prompt_pin_int], 5)); // Send the pin value

                    if (time - old_time > interval)
                    {
                        Serial.println();
                        Serial.print("pin=");
                        Serial.print(prompt_pin_int);
                        Serial.print(" ");
                        Serial.print("count=");
                        Serial.print(tmpcount);
                        Serial.print(" in ");
                        Serial.print(time - old_time);
                        Serial.print("us");
                        Serial.print(" avg=");
                        Serial.print(((time - old_time) / tmpcount));
                        Serial.print("us");
                        Serial.println();
                        delay(100);
                        tmpcount = 0;
                        time = micros();
                        old_time = time;
                    }

                }
                pinMode(prompt_pin_int, INPUT);
                bit_clear(arduino1280DigitalBitMap[prompt_pin_int], 5);
                return 1;

                break;


            case 's':
                pinMode(prompt_pin_int, OUTPUT);
                while (Serial.read() == -1)
                {
                    time = micros();
                    tmpcount++;
                    bit_flip(arduino1280DigitalBitMap[prompt_pin_int], 5);
                    digitalWrite(prompt_pin_int, (bit_get(arduino1280DigitalBitMap[prompt_pin_int], 5))); // Send the pin value

                    if (time - old_time > interval)
                    {
                        Serial.println();
                        Serial.print("pin=");
                        Serial.print(prompt_pin_int);
                        Serial.print(" ");
                        Serial.print("count=");
                        Serial.print(tmpcount);
                        Serial.print(" in ");
                        Serial.print(time - old_time);
                        Serial.print("us");
                        Serial.print(" avg=");
                        Serial.print(((time - old_time) / tmpcount));
                        Serial.print("us");
                        Serial.println();
                        delay(100);
                        tmpcount = 0;
                        time = micros();
                        old_time = time;
                    }

                }
                pinMode(prompt_pin_int, INPUT);
                bit_clear(arduino1280DigitalBitMap[prompt_pin_int], 5);
                return 1;

                break;



            default:
                Serial.print("What the Hell does this mean?");// for debug           
                // if nothing else matches, do the default
                // default is optional
            }
        }

        ////////////////////////////////////////////////////////////////  
        //End process command and pin number
        ////////////////////////////////////////////////////////////////
    }
    PausePressKey();
    return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// END int ProcessCommandLine()
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void blinker(int pin)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//http://code.google.com/p/digitalwritefast/downloads/list

void blinker(int pin)
{
    Serial.print("b");
    Serial.print(pin, DEC); // Send the pin value
    if (bit_get(arduino1280DigitalBitMap[pin], 3))    //turns off blink on 1 pin
    {
        bit_clear(arduino1280DigitalBitMap[pin], 3);
        bit_clear(arduino1280DigitalBitMap[pin], 5);
        bit_clear(arduino1280DigitalBitMap[pin], 7);
        Serial.println("=BLINK OFF"); // Add a space separator      
        digitalWrite(pin, LOW); //Sets the state of the pin to low to avoid Pullup state      
        pinMode(pin, INPUT);//turns pin state to input
        digitalWrite(pin, LOW); //Sets the state of the pin to low to avoid Pullup state

    }
    else if (!bit_get(arduino1280DigitalBitMap[pin], 3))//turns on blink
    {
        Serial.print("=BLINK ON"); // Add a space separator      
        Serial.println(); // Terminate message  
        globalblinkstate = ON; //turns on blink 
        bit_set(arduino1280DigitalBitMap[pin], 3);
        bit_set(arduino1280DigitalBitMap[pin], 5);
        bit_set(arduino1280DigitalBitMap[pin], 7);
        pinMode(pin, OUTPUT);//turns pin state to output
        digitalWrite(pin, bit_get(arduino1280DigitalBitMap[pin], 3)); //Sets the state of the pin to ledstat
        //turns off globalblinkstate
    }
    for (int i = 0; i < arduino1280DigitalBitMapLength; i++)
    {
        if (bit_get(arduino1280DigitalBitMap[i], 3))
        {
            globalblinkstate = ON;
            return;
        }
    }
    globalblinkstate = OFF;
    return;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void display_freq(int pin)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void display_freq(int pin)
{
    long freq = 0;
    Serial.print("f ");
    Serial.print(" "); // Add a space separator
    Serial.print(pin);
    Serial.print(" Displays freq till keypress "); // Add a space separator
    Serial.println();
    //Do not forget to configure your pin as input!
    pinMode(pin, INPUT);//turns pin state to input

    int tmpcount;
    Serial.print(" Pin changes read as count if no count then freq measurement aborted"); // Add a space separator
    Serial.println();
    while (tmpcount < 100)
    {
        freq = freq + digitalRead(pin);
        Serial.print(freq);
        Serial.println();
        tmpcount++;
    }
    if (freq > 30)
    {
        Serial.print(" Displays freq till keypress "); // Add a space separator
        Serial.println();
        while (Serial.read() == -1)
        {
            freq = getFrequency(pin);
            Serial.print(pin);
            Serial.print("=");
            Serial.print(freq);
            Serial.print("hz");
            Serial.println();
        }
        Serial.println();
    }

}

//For shorter delays use assembly language call 'nop' (no operation). 
//Each 'nop' statement executes in one machine cycle (at 16 MHz) yielding a 62.5 ns (nanosecond) delay.
void delaynanosecond()
{
    __asm__("nop\n\t");
    //  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");  \\ gang them up like this
}


void printHex4(int value)
{
    //Print a hexadecimal number at 4 places
    for (int a = sizeof(value) * 2 - 1; a >= 0; a--)
    {
        Serial.print(("0123456789ABCDEF"[((value >> a * 4) & 0xF)]));
    }
}
void printHex2(int value)
{
    //Print a hexadecimal number at 2 places
    for (int a = sizeof(value) / 1 - 1; a >= 0; a--)
    {
        Serial.print(("0123456789ABCDEF"[((value >> a * 4) & 0xF)]));
    }
}
void printHex1(int value)
{
    //int padding 2 2=1 or 1 2=2 
    //Print a hexadecimal number at 1 place
    for (int a = sizeof(value) / 2 - 1; a >= 0; a--)
    {
        Serial.print(("0123456789ABCDEF"[((value >> a * 4) & 0xF)]));
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void draw_analog(int speeder)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void draw_analog(int speeder)
{
    for (int i = analog_pins_min; i < analog_pins_max; i++)
    {
        if (speeder == FAST) {
            printHex2((analogRead(i) / 4));
            Serial.print(" ");
            //Serial.print(analogarray[i]/4 ,HEX); 

            //if(i < (analog_pins_max-1))  {
            //	  Serial.print(",");  }
        }
        if (speeder == FASTER) {

            //Serial.print( (analogRead(i)/68) ,HEX);
            printHex1((analogRead(i) / 68));
            Serial.print(" ");

        }
        else if (speeder == SLOW) {
            Serial.print(voltage(analogRead(i)), 2);
            if (i < (analog_pins_max - 1)) {
                Serial.print(" ");
            }
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void draw_digital(int speeder)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void draw_digital(int speeder)
{
    for (int i = digital_pins_min; i < digital_pins_max; i++)
    {
        if (speeder == FAST)
        {
            Serial.print(digitalRead(i));
        }
        if (speeder == FASTER)
        {
            Serial.print(digitalReadFast(i));
        }
        else if (speeder == SLOW)
        {
            Serial.print(digitalRead(i));
        }
    }
}



int digitalReadFast(uint8_t pin)
{
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    if (*portInputRegister(port) & bit) return HIGH;
    return LOW;
}
void digitalWriteFast(uint8_t pin, uint8_t val)
{
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    volatile uint8_t* out;
    out = portOutputRegister(port);
    if (val == LOW) {
        uint8_t oldSREG = SREG;
        *out &= ~bit;
        SREG = oldSREG;
    }
    else {
        uint8_t oldSREG = SREG;
        *out |= bit;
        SREG = oldSREG;
    }
}
































////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
// Start Re-Useble Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////  
//int create_ascii_number(int a,int b, int c)
////////////////////////////////////////////////////////////////
int create_ascii_number(int a, int b, int c)
{
    if (is_ASCII_number(c))//the third int value if present
    {
        a = a - '0';
        b = b - '0'; //the second int value if present
        c = c - '0'; //the third int value if present     
        a = (a * 100) + (b * 10) + c;
        return a;
    }
    else if (is_ASCII_number(b))//the second int value if present
    {
        a = a - '0';
        b = b - '0';
        a = (a * 10) + b;
        return a;
    }
    else if (is_ASCII_number(a))//first number if present
    {
        a = a - '0';
        return a;
    }
    else
    {
        return -9999;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void printBits(byte myByte){
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void printBits(byte myByte) {
    for (byte mask = 0x80; mask; mask >>= 1) {
        if (mask & myByte)
            Serial.print('1');
        else
            Serial.print('0');
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//float voltage(float val, int dp)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float voltage(float val, int dp)
{
    val = val * .005;
    return int(val * pow(10, dp)) / pow(10, dp);
}

float voltage(float voltage)
{
    //Convert to actual voltage (0 - 5 Vdc)
    voltage = (voltage / 1024) * 5.0;
    return voltage;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//int is_ASCII_number(int inByte)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int is_ASCII_number(int inByte)
{
    if ((inByte >= 48 && inByte <= 57))
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//int is_ASCII_letter((int inByte)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int is_ASCII_letter(int inByte)
{
    if ((inByte >= 65 && inByte <= 90) || (inByte >= 97 && inByte <= 122))
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//long getFrequency(int pin) {
//http://tushev.org/articles/electronics/43-measuring-frequency-with-arduino
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
long getFrequency(int pin) {
#define SAMPLES 4096
    long freq = 0;
    for (unsigned int j = 0; j < SAMPLES; j++)
    {
        freq += 500000 / pulseIn(pin, HIGH, 250000);
    }
    return freq / SAMPLES;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void print_milli_timer()
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void print_milli_timer()
{
    Serial.println();
    Serial.print("time=");
    Serial.print(millis());
    Serial.print("ms");
    Serial.println();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void print_micro_timer()
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void print_micro_timer()
{
    Serial.println();
    Serial.print("time=");
    Serial.print(micros());
    Serial.print("us");
    Serial.println();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void read_cap(int chargePin)
//http://arduino.cc/it/Tutorial/CapacitanceMeter
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//read_cap(arduinoArray[0][2],arduinoArray[0][0],resistorValueLI)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_cap(int chargePin, int analogPin, int resistorValue)
{
    unsigned long startTime;
    unsigned long elapsedTime;
    float microFarads;// floating point variable to preserve precision, make calculations
    float nanoFarads;
    digitalWrite(chargePin, HIGH);  // set chargePin HIGH and capacitor charging
    startTime = millis();

    //loop needs 2 sec timeout	
    while (analogRead(analogPin) < 648) {       // 647 is 63.2% of 1023, which corresponds to full-scale voltage 
        //loop needs 2 sec timeout	
        if ((millis() - startTime) > 2000) {
            Serial.println("Timeout");
            return;
        }

    }

    elapsedTime = millis() - startTime;
    // convert milliseconds to seconds ( 10^-3 ) and Farads to microFarads ( 10^6 ),  net 10^3 (1000)  
    microFarads = ((float)elapsedTime / resistorValue) * 1000;
    Serial.print("Capacitance: ");     // print the value to serial port
    Serial.print(microFarads);       // print the value to serial port
    Serial.println(" microFarads");         // print units and carriage return

    Serial.print(elapsedTime);       // print the value to serial port
    Serial.print(" mS    ");         // print units and carriage return


    if (microFarads > 1) {
        Serial.print((long)microFarads);       // print the value to serial port
        Serial.println(" microFarads");         // print units and carriage return
    }
    else
    {
        // if value is smaller than one microFarad, convert to nanoFarads (10^-9 Farad). 
        // This is  a workaround because Serial.print will not print floats
        nanoFarads = microFarads * 1000.0;      // multiply by 1000 to convert to nanoFarads (10^-9 Farads)
        Serial.print((long)nanoFarads);         // print the value to serial port
        Serial.println(" nanoFarads");          // print units and carriage return
    }

    digitalWrite(chargePin, LOW);          // set discharge pin LOW 
    pinMode(chargePin, INPUT);            // set discharge pin back to input
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  void CreatelCommandLine(int keypress) 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CreateCommandLine(int keypress)
{
    int result;

    if ((keypress > -1) && (keypress != 13))
    {
        if ((characterToStringPointer > 0) && (keypress == 8))// Backspace
        {
            commandline[--characterToStringPointer] = 0;
        }
        else if ((keypress) && (characterToStringPointer < commandline_max_len) && (keypress != 13))
        {
            commandline[characterToStringPointer] = keypress;//    'Write character to commandline
            //Serial.print(commandline[characterToStringPointer]);//prints the character just read
            Serial.print((char)commandline[characterToStringPointer]);//prints the character just read

            //Serial.print(receivedchar); //prints the character just read for debug
            characterToStringPointer++;
        }
    }
    else if (keypress == 13)
    {
        //characterToStringPointer now contains length of command line
        //add 0 at end and turn it into a string!
        characterToStringPointer++;
        commandline[characterToStringPointer] = 0;

        ///////////////////////////////////////////
        //Display Command line for debug
        ///////////////////////////////////////////
        debug_commandline(FALSE);

        ///////////////////////////////////////////
        //Run Command line
        ///////////////////////////////////////////
        result = ProcessCommandLine();
        printerror(result);// for debug

        ///////////////////////////////////////////
        //Reset Command line
        ///////////////////////////////////////////
        ResetCommandLine();
    }

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void ResetCommandLine()
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ResetCommandLine()
{
    characterToStringPointer = 0;
    for (int i = 0; i < commandline_max_len; i++)
    {
        commandline[i] = 0;;//'filess it up with commandline_max_len 0's  
    }
}
////////////////////////////////////////////////////////////////  
//int isdigitalpinvalid(int pin)
////////////////////////////////////////////////////////////////
int isdigitalpinvalid(int pin)
{
    if (pin >= digital_safety_pins_min && pin <= digital_safety_pins_max)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}
////////////////////////////////////////////////////////////////  
//int isanalogpinvalid(int pin)
////////////////////////////////////////////////////////////////
int isanalogpinvalid(int pin)
{
    if (pin >= analog_safety_pins_min && pin <= analog_safety_pins_max)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// void debug_commandline(int tmp)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void debug_commandline(int tmp)
{
    int array_ending = 0;// for debug  
    if (tmp)
    {
        Serial.println();
        Serial.println("****************************************");
        Serial.println("Commandline Debug debug_commandline(TRUE); ");
        Serial.println("****************************************");// for debug
        Serial.println("commandline length=");// for debug
        while (commandline[++array_ending] != 0); // find ending // for debug   
        Serial.println(array_ending);// for debug
        Serial.print("characterToStringPointer=");// for debug
        Serial.println(characterToStringPointer);// for debug
        Serial.println("TOKENS=");// for debug
        for (int countmux = 0; countmux < array_ending; countmux++)// for debug
        {// for debug
            Serial.print(countmux); // for debug        
            Serial.print("=[");  // for debug
            Serial.print(commandline[countmux]); // for debug 
            Serial.print("]"); // for debug   
            Serial.println(); // for debug       
        }// for debug
        Serial.println(); // for debug       
        PausePressKey();
    }

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void printerror(int result)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void printerror(int result)
{
    if (result != 0)
    {
        Serial.println(); // for debug    
        Serial.print("ERROR=");// for debug
        Serial.print(result);// for debug
        Serial.println(); // for debug       
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void digital_pin_change(int pin,int state,int IN_OUT)  SLOW Speed but has error correction
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void digital_pin_change(int pin, int state, int IN_OUT)
{
    if (IN_OUT == INPUT) { digitalWrite(pin, LOW); }//error correction for possible pullup state when changing pin states
    pinMode(pin, IN_OUT); //Changes pin state to Qutput
    digitalWrite(pin, state); //Sets the state of the pin 
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void digital_pullup(int pin)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void digital_pullup(int pin)
{
    pinMode(pin, INPUT);
    digitalWrite(pin, HIGH); //Sets the state of the pin  
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void read_all_digitalpins()
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_all_digitalpins()
{
    for (int countmux = digital_pins_min; countmux < digital_pins_max; countmux++)
    {
        //Read and store the input value at a location in the array
        digitalarray[countmux] = digitalRead(countmux);
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void read_all_analogpins()
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_all_analogpins()
{
    for (int countmux = analog_pins_min; countmux < analog_pins_max; countmux++)
    {
        //Read and store the input value at a location in the array
        analogarray[countmux] = analogRead(countmux);
    }
}
//http://arduino.cc/playground/Main/FindText
//Little function that searches for a given string within another string. If the search string is found its position is returned, otherwise -1 is returned.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//int find_text(String needle, String haystack) {
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int find_text(String needle, String haystack) {
    int foundpos = -1;
    for (int i = 0; (i < haystack.length() - needle.length()); i++) {
        if (haystack.substring(i, needle.length() + i) == needle) {
            foundpos = i;
        }
    }
    return foundpos;
}
























////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
// END Re-Useble Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////


