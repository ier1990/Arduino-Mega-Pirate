# Arduino Mega Pirate
 Arduino Logic State Analyzer<br>
 Menu<br>  
    Press ?  for Available Commands<br>
    Press /  to create Commandline<br>
    <br>
    -----------------Running Commandline--------------------<br>
    Press g  Faster Scan A=/.3v  resolution<br>
    Press f  Fast Scan   A=.02v  resolution<br>
    Press s  Slow Scan   A=.005v resolution<br>
    Press t  Displays miliseconds since program start<br>
    Press a  Analog Scan<br>
    Press d  Digital Scan<br>
    Press m  Available Memory<br>
    Press i  I2c Scanner & arduino1280DigitalBitMap Array<br>
    Press z  Register Port F&K Scan<br>
    Press y  Register Port E&H Scan<br>
    Press x  ALL Port Port A-L Scan<br>
    Press e  8 Bit Shift Register Pins:clock" + String(clockPin) + ",enable:" + String(enablePin10k) + ",data:" + String(dataPin10k));<br>    
    Press w  8 Bit Shift Register Pins:clock" + String(clockPin) + ",enable:" + String(enablePinvoltmod) + ",data:" + String(dataPin10k));<br>
    Press p  Prints all digitalPinToBitMask to digitalPinToPort " + String(arduino1280DigitalBitMapLength) + " Digital Pins<br>
    <br>    
    -----------------/W WRITE------------------------------<br>
    wb#     Makes pin# Output and BLINKS it HIGH/LOW<br>
    //WriteBlink runs in main loop<br>
    df/ds#  BLINKS ~8khz Runs 1 sec then pauses for 100 millisecs for Serial.read<br>
    //DigitalWrite     = /ds# = 1366us<br>
    //DigitalWriteFast = /df# = 62us !!!!!!!!!!!!!!!!!!!!!!!MUCH BETTER<br>
    //No Serial.Read or TX for 1 sec while digitalWriteFast 1/0 pin with same timing loop takes 62us per loop <br>
    wh#     Makes pin# Output as HIGH <br>
    wu#     Makes pin# PullUp <br>
    wl#     Makes pin# Output as LOW<br>
    wi#     Makes pin# as Input<br>
    <br>
    -----------------/R READ------------------------------<br>
    rx      1sec TX 115kbaud(12kchars48kbitstatesMax) using HEX Protacol<br>
    //One Char takes 1/12000 = 80us freq so 12k is the freq aka samples per second not complete cycles !<br>
    //Samples 1 sec then pauses for 100 millisecs for Serial.read<br>
    rd#     Read digital pin# value digitalRead(<br>
    //DigitalRead      = /rd# = 80us  NoSerial.print=(14us ~ 71k reads per sec)       <br>
    re#     Read digital pin# value digitalReadFast(<br>
    //DigitalReadFast  = /re# = 80us  NoSerial.print=( 8us ~125k reads per sec)<br>
    ra#     Read analog  pin# value A=.3v  resolution<br>
    //analogRead() with No Serial.Read or TX for 1 sec  8300 count at 120us per read      <br>
    //analogRead() with    Serial.Read or TX for 1 sec  6914 count at 144us per read<br>
    //With FASTADC with    Serial.Read or TX for 1 sec 11768 count at  84us per read  !!!!!!!!!!!!!!!!!!!!!!!MUCH BETTER<br>
    //With FASTADC with No Serial.Read or TX for 1 sec 36948 count at  27us per read  <br>
    -----------------/ Expermental or Broke-----------------<br>
    rf#     Measures Freq pin# Reads<br>
    b$      $ = letter of register port to read a,b,c,d,e,f,g,h,j,k,l<br>    
    rc#     Read Capacitance value analog pin# 0-15, takes ~ 2secs <br>
<br>
Complied on 1280 & 2560

    
