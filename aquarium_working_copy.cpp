
// include the library code:
#include <LiquidCrystal.h>
#include <Time.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>
#include <OneWire.h>
#include <Wire.h>
 
#define PHADDRESS 0x4D
int RoomTempI2CAddress = B1001011;
 
float volt4 = 0.977;
float volt7 = 0.6888;
float calibrationTempC = 24;

/* ******** Ethernet Card Settings ******** */
// Set this to your Ethernet Card Mac Address
byte mac[] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF }; 

/* ******** NTP Server Settings ******** */
/* us.pool.ntp.org NTP server 
   (Set to your time server of choice) */
//IPAddress timeServer(132, 163, 4, 101);
//IPAddress timeServer(96, 126, 105, 86);
IPAddress timeServer(62, 2, 85, 186);


//IPAddress timeServer(130, 88, 200, 6);

/* Set this to the offset (in seconds) to your local time
   This example is GMT + 3 */
const long timeZoneOffset = 7250L;   

/* Syncs to NTP server every 10 minutes */
unsigned int ntpSyncTime = 600;         

// local port to listen for UDP packets
unsigned int localPort = 8888;
// NTP time stamp is in the first 48 bytes of the message
const int NTP_PACKET_SIZE= 48;      
// Buffer to hold incoming and outgoing packets
byte packetBuffer[NTP_PACKET_SIZE];  
// A UDP instance to let us send and receive packets over UDP
EthernetUDP Udp;                    
// Keeps track of how long ago we updated the NTP server
unsigned long ntpLastUpdate = 0;    
// Check last time clock displayed (Not in Production)
time_t prevDisplay = 0;             

/******************************************************************/

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8,   9,  4,  5,  6,  7);

// Initialize Arduino Ethernet Client
EthernetClient client;
boolean lastConnected = false;
int failedCounter = 0;

// defining the working items
#define LIGHTS 22  // digital
#define MOONLIGHTS 24 // digital
#define CO2 26 // digital
#define FANS  28 // digital
#define HEATER 30 // digital
#define LIGHTS_PWM 2// digital
#define MOONLIGHTS_PWM 3 // digital
#define PWM_MAX 255
#define PWM_MIN 0

const unsigned long HOUR = 60 * 60;
const unsigned long MINUTE = 60;
const int TARGET_BRIGHTNESS = 255;
const int DESIRED_BRIGHTNESS = (255 * 2) / 10;


float temperature = 0;

// according to my aquarium water's dkH = 12 
const float MINIMUM_PH = 7.00;
const float MAXIMUM_PH = 7.10;

float pH = 0;
int co2Check = 0;

OneWire  ds(A8); // watertemp

byte water[8] = //icon for water droplet
{
    B00100,
    B00100,
    B01010,
    B01010,
    B10001,
    B10001,
    B10001,
    B01110,
};

byte celsiusSymbol[8] = //icon for water droplet
{
    B01000,
    B10100,
    B01000,
    B00011,
    B00100,
    B00100,
    B00100,
    B00011,
};

void welcomeLcd(){
    // set up the LCD's number of columns and rows: 
    lcd.begin(16, 2);
    // Print a message to the LCD.
    lcd.print("Mario V1.0");
    delay(800);
    lcd.clear();
}

void displaySystem(float temp, int co2Status, float ph){

  lcd.clear();
  lcd.setCursor(0,0);

  //write temp symbol
  lcd.write(byte(1));
  lcd.write(":");
  printFloat(temp,1);
  //lcd.print(temp);
  addCelciusSymbol();

  lcd.setCursor(9,0);
  lcd.print("CO2:");
  if(co2Status == 1) lcd.write("ON ");
  else if(co2Status == 0) lcd.write("OFF");
  
  lcd.setCursor(0,2);
  lcd.print("pH:");
  lcd.print(ph, 2);
  //lcd.print(dayShortStr(weekday()));
  lcd.setCursor(8,2);
  print2digits(hour());
  lcd.write(":");
  print2digits(minute());
  lcd.write(":");
  print2digits(second());
  addCelciusSymbol(); 
  lcd.setCursor(10,2);
  
}

void addCelciusSymbol(){

  // temperature symbol-2
  lcd.print((char)223);
  lcd.print("C");  

}

void print2digits(int number) {
  // Output leading zero
  if (number >= 0 && number < 10) {
    lcd.write('0');
  }
  lcd.print(number);
}
  // DS18B20 - water temp prob.
float checkWaterTemp(){

  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;
  
  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(1000);
    ds.reset();
    return 0;
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return 0;
  }
 
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);  // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
 
  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  Serial.print("  Temperature = ");
  Serial.print(celsius);
  //Serial.print(" Celsius, ");
  
  return celsius;
}

//Heater Control 
void heaterControl(float temp){
  
  if(temp > 0 && temp <= 28){ //>0 to avoid -999.00*C error   
    digitalWrite(HEATER, HIGH);
    //Serial.println("Heating..."); 
  } else if(temp > 28 && temp < 50){ //<0 to avoid -999.00*C error
    digitalWrite(HEATER, LOW);
    //Serial.println("Heater OFF");
  } else {
    // just in case
    digitalWrite(HEATER, LOW);
    //Serial.println("Heater OFF");
  }
  
}

//Cooling Fan Control
void fanControl(int open){

  if(open == 1){
    digitalWrite(FANS, HIGH);
    //Serial.println("FANS are opened...");

  } else if( open == 0){
    digitalWrite(FANS, LOW);
    //Serial.println("FANS are closed...");    
  }

}

void setup() {

  // Setting welcome screen
  welcomeLcd(); 
  
  Serial.begin(9600);

  getTimeNTP(true);

  Wire.begin();

  // set system defaults
  setDefaults();
    
}

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void setDefaults(){

  // setting pins to output mode.
  pinMode(FANS, OUTPUT);       
  pinMode(HEATER, OUTPUT);
  pinMode(LIGHTS, OUTPUT);
  pinMode(CO2, OUTPUT);
  pinMode(MOONLIGHTS, OUTPUT);

  digitalWrite(FANS, LOW);
  digitalWrite(LIGHTS, LOW);
  digitalWrite(MOONLIGHTS, LOW);
  digitalWrite(HEATER, LOW);

  // water symbol
  lcd.createChar(1, water);

  //celcius symbol
  lcd.createChar(2, celsiusSymbol);

  float passTemp = checkWaterTemp();
  if( passTemp > 1)
    temperature = passTemp;

  pH = getPhValue();
  co2Check = controlCO2WithPH(pH);

}

void controlMoonLeds(unsigned long myTime){

  //byte first = brightness(myTime, 17*HOUR, 17*HOUR+30*MINUTE, 15);
  byte second = brightness(myTime, 15*HOUR, 2*HOUR+45*MINUTE, 1);

  if(second == 0){
    digitalWrite(MOONLIGHTS, LOW);
  } else{
      digitalWrite(MOONLIGHTS, HIGH);
  }
}

int controlLeds(unsigned long myTime){

  int openFan = 0;

  byte first = brightness(myTime, 9*HOUR, 12*HOUR+45*MINUTE, 15);
  //byte second = brightness(myTime, 14*HOUR, 16*HOUR+45*MINUTE, 15);
  byte third = brightness(myTime, 19*HOUR, 23*HOUR+45*MINUTE, 15);
  
  //revert the pwm value because of the pwm curcuit
  first = map(first, PWM_MIN, PWM_MAX, PWM_MAX, DESIRED_BRIGHTNESS);
  //second = map(second, PWM_MIN, PWM_MAX, PWM_MAX, DESIRED_BRIGHTNESS);
  third = map(third, PWM_MIN, PWM_MAX, PWM_MAX, DESIRED_BRIGHTNESS);

  if(first == PWM_MIN || third == PWM_MIN){
    digitalWrite(LIGHTS, HIGH);
    analogWrite(LIGHTS_PWM, DESIRED_BRIGHTNESS);
    openFan = 1;

  } else if(first == PWM_MAX  && third == PWM_MAX){
      digitalWrite(LIGHTS, LOW);
      openFan = 0;

  } else{
      digitalWrite(LIGHTS, HIGH);
      if(first < third){
        analogWrite(LIGHTS_PWM, first);
      }
      else if(third < first){
        analogWrite(LIGHTS_PWM, third);
      }
      openFan = 1;
  }
  return openFan;
}

int controlCO2WithPH(float myPH){

  int openCo2 = 0;

  if (myPH >= MAXIMUM_PH){
    //open co2
    digitalWrite(CO2, HIGH);
    openCo2 = 1;
  } else if(myPH <= MINIMUM_PH){
      //close co2
      digitalWrite(CO2, LOW);
  } else {
      //close co2
      digitalWrite(CO2, LOW);
  }

  return openCo2;
}

int controlCO2(unsigned long myTime){

  int openCo2 = 0;

  if ((myTime >= 10*HOUR+30*MINUTE  && myTime <= 14*HOUR+30*MINUTE) || (myTime >= 18*HOUR+30*MINUTE && myTime <= 23*HOUR+15*MINUTE) ){
    //open co2
    digitalWrite(CO2, HIGH);
    openCo2 = 1;
  } else{
      //close co2
      digitalWrite(CO2, LOW);
  }

  return openCo2;
}

byte brightness(unsigned long lastDaySeconds, unsigned long fadeUpStart, unsigned long fadeDownStart, unsigned long fadeTime)
{
  
  if (lastDaySeconds >= fadeUpStart + MINUTE*fadeTime && lastDaySeconds <= fadeDownStart){
    return TARGET_BRIGHTNESS;
  }

  // Dawn:  fade up the light
  if (lastDaySeconds >= fadeUpStart && lastDaySeconds <= fadeUpStart + fadeTime*MINUTE)  // Fading up
  {
    unsigned long seconds = lastDaySeconds - fadeUpStart;  // Number of seconds into the fade time 
    return TARGET_BRIGHTNESS * seconds / (MINUTE*fadeTime);  // Fade up based on portion of interval completed.
  }

  // Evening: Fade down the light
  if (lastDaySeconds >= fadeDownStart && lastDaySeconds <= fadeDownStart + fadeTime*MINUTE)  // Fading down
  {
    unsigned long seconds = (fadeDownStart + (MINUTE*fadeTime)) - lastDaySeconds;  // Number of seconds remaining in the fade time 
    return TARGET_BRIGHTNESS * seconds / (MINUTE*fadeTime);  // Fade down based on portion of interval left.
  }

  // The remaining times are night and the lights is off
  return 0;  // Shouldn't get here
}

void loop() {

  unsigned long today = hour() * HOUR + minute() * MINUTE + second();

  if(now() % 3 == 0){
    float temp = checkWaterTemp();
    if(temp > 0 && temp < 40)
      temperature = temp;
  }

  //check the water tempature and pH every 4 secs.
  if(now() % 4 == 0){
    float pHTemp = getPhValue();
    if(pHTemp > 0 && pHTemp < 14)
      pH = pHTemp;
  }

  //control the water pH every 2.5 minutes.
  if (now() % 150 == 0){
    co2Check = controlCO2WithPH(pH);
  }

  //int co2Check = controlCO2(today); 
  heaterControl(temperature);
  controlLeds(today);
  controlMoonLeds(today);

  // Update the time via NTP server as often as the time you set at the top
  if(now()-ntpLastUpdate > ntpSyncTime) {
    int trys=0;
    while(!getTimeAndDate(false) && trys<10){
      trys++;
    }
    if(trys<10){
      Serial.println("ntp server update success");
    }
    else{
      Serial.println("ntp server update failed");
    }
  }

  // Display the time if it has changed by more than a second.
  if(now() != prevDisplay){
    prevDisplay = now();
    displaySystem(temperature, co2Check, pH);  
  }


  // Check if Arduino Ethernet needs to be restarted
  if (failedCounter > 3 ) {startEthernet();}
    lastConnected = client.connected();   
  
}


void startEthernet(){

  client.stop();
  Serial.println("Connecting Arduino to network...");
  Serial.println();  
  delay(1000);
  // Connect to network amd obtain an IP address using DHCP
  if (Ethernet.begin(mac) == 0){
      Serial.println("DHCP Failed, reset Arduino to try again");
      Serial.println();
  } else{
      Serial.println("Arduino connected to network using DHCP");
      Serial.println();
  }

}


void printFloat(float value, int places) {
  // this is used to cast digits 
  int digit;
  float tens = 0.1;
  int tenscount = 0;
  int i;
  float tempfloat = value;

    // make sure we round properly. this could use pow from <math.h>, but doesn't seem worth the import
  // if this rounding step isn't here, the value  54.321 prints as 54.3209

  // calculate rounding term d:   0.5/pow(10,places)  
  float d = 0.5;
  if (value < 0)
    d *= -1.0;
  // divide by ten for each decimal place
  for (i = 0; i < places; i++)
    d/= 10.0;    
  // this small addition, combined with truncation will round our values properly 
  tempfloat +=  d;

  // first get value tens to be the large power of ten less than value
  // tenscount isn't necessary but it would be useful if you wanted to know after this how many chars the number will take

  if (value < 0)
    tempfloat *= -1.0;
  while ((tens * 10.0) <= tempfloat) {
    tens *= 10.0;
    tenscount += 1;
  }


  // write out the negative if needed
  if (value < 0)
    lcd.print('-');

  if (tenscount == 0)
    lcd.print(0, DEC);

  for (i=0; i< tenscount; i++) {
    digit = (int) (tempfloat/tens);
    lcd.print(digit, DEC);
    tempfloat = tempfloat - ((float)digit * tens);
    tens /= 10.0;
  }

  // if no places after decimal, stop now and return
  if (places <= 0)
    return;

  // otherwise, write the point and continue on
  lcd.print('.');  

  // now write out each decimal place by shifting digits one by one into the ones place and writing the truncated value
  for (i = 0; i < places; i++) {
    tempfloat *= 10.0; 
    digit = (int) tempfloat;
    lcd.print(digit,DEC);  
    // once written, subtract off that digit
    tempfloat = tempfloat - (float) digit; 
  }

}

// Do not alter this function, it is used by the system
int getTimeAndDate(bool firstTime) {
   int flag=0;
   Udp.begin(localPort);
   sendNTPpacket(timeServer);
   delay(1000);
   if (Udp.parsePacket()){
     Udp.read(packetBuffer,NTP_PACKET_SIZE);  // read the packet into the buffer
     unsigned long highWord, lowWord, epoch;
     highWord = word(packetBuffer[40], packetBuffer[41]);
     lowWord = word(packetBuffer[42], packetBuffer[43]);  
     epoch = highWord << 16 | lowWord;
     epoch = epoch - 2208988800 + timeZoneOffset;
     flag=1;
     setTime(epoch);
     ntpLastUpdate = now();
   } else{
    if(firstTime){
      delay(5000);
      getTimeNTP(true);
    }
   }
   return flag;
}

// Do not alter this function, it is used by the system
unsigned long sendNTPpacket(IPAddress& address){
  memset(packetBuffer, 0, NTP_PACKET_SIZE); 
  packetBuffer[0] = 0b11100011;
  packetBuffer[1] = 0;
  packetBuffer[2] = 6;
  packetBuffer[3] = 0xEC;
  packetBuffer[12]  = 49; 
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;      
  Udp.beginPacket(address, 123);
  Udp.write(packetBuffer,NTP_PACKET_SIZE);
  Udp.endPacket(); 
}

void getTimeNTP(bool firstTime){

  // Ethernet shield and NTP setup
   int i = 0;
   int DHCP = 0;
   DHCP = Ethernet.begin(mac);

   //Try to get dhcp settings 120 times before giving up
   while( DHCP == 0 && i < 120){
     delay(1000);
     DHCP = Ethernet.begin(mac);
     i++;
   }

   if(!DHCP){

    Serial.println("DHCP FAILED");
    resetFunc();  

   }
   Serial.println("DHCP Success");

   //Try to get the date and time
   int trys=0;
   while(!getTimeAndDate(firstTime) && trys<100) {
     trys++;
   }

}



//PH Calculation Start
float getPhValue(){

  Serial.print("avgMeasuredPH-");
  SetRoomTemperataureResolutionBits(12);//12 bits room temp resolution in celcius
  
  int sampleSize = 500;
  
  double avgMeasuredPH = 0;
  double avgRoomTempC = 0;
  double avgPHVolts = 0;
  double avgRoomTemperatureCompensatedMeasuredPH = 0;
  
  double tempAdjusted4;
  
  int x;
  for(x=0;x< sampleSize;x++)
  {
  
  double phVolt = getPHVolts();    
  tempAdjusted4 = adjustPHBasedOnTemp(4,calibrationTempC);
  double voltsPerPH = (abs(volt7-volt4)) / (7-tempAdjusted4);
  
  double realPHVolt = (volt7 - phVolt);
  double phUnits = realPHVolt / voltsPerPH;
  double measuredPH = 7 + phUnits;
 
  double roomTempC =  getRoomTemperatureC();
  double roomTempCompensatedMeasuredPH = adjustPHBasedOnTemp(measuredPH,roomTempC); 
  
  avgMeasuredPH+=measuredPH;
  avgRoomTemperatureCompensatedMeasuredPH+=roomTempCompensatedMeasuredPH;
  avgRoomTempC+=roomTempC;
  avgPHVolts += phVolt;
    
  
  }
  
  avgMeasuredPH/=sampleSize;
  avgRoomTemperatureCompensatedMeasuredPH/=sampleSize;
  avgRoomTempC/=sampleSize;
  avgPHVolts/=sampleSize;
  
  Serial.print("avgMeasuredPH-");
  Serial.print(avgMeasuredPH,4);
  Serial.print(" roomTempCompensatedPH-");
  Serial.print(avgRoomTemperatureCompensatedMeasuredPH,4);
  Serial.print(" avgRoomtTempC-");
  Serial.print(avgRoomTempC,4);
  Serial.print(" avgPhVolts-");
  Serial.print(avgPHVolts,4);
  
  Serial.print(" 7CalVolts-");
  Serial.print(volt7,4);
  Serial.print(" 4CalVolts-");
  Serial.print(volt4,4);    
  Serial.print(" 4CalTempAdjusted-");
  Serial.println(tempAdjusted4,4);
 
  return avgMeasuredPH;

}

 
float adjustPHBasedOnTemp(float PH, float temp)
{
   // http://www.omega.com/Green/pdf/pHbasics_REF.pdf
   // When the temperature is other than 25degC and the ph is other than 7
   // the temperature error is 0.03ph error/ph unit/10degC
   // which means error = 0.03*(ph away from 7)*(tempdiffC/10)
   
    float phDifference = abs(PH-7);
    float tempDifferenceC = abs(temp-25);
    float phAdjust = (0.03*phDifference)*(tempDifferenceC/10);
    
    if(PH>7 && temp<25)
      phAdjust=phAdjust;
 
    if(PH>7 && temp>25)
      phAdjust=phAdjust*-1;
 
    if(PH<7 && temp>25)
      phAdjust=phAdjust;
 
    if(PH<7 && temp<25)
      phAdjust=phAdjust*-1;
 
    float tempAdjustedPH = PH + phAdjust;
    return tempAdjustedPH;
}
 
 
 
double getPHVolts()
{
  byte ad_high;
  byte ad_low;
  
  Wire.requestFrom(PHADDRESS, 2);        //requests 2 bytes
  while(Wire.available() < 2);         //while two bytes to receive
    
  ad_high = Wire.read();           
  ad_low = Wire.read();
  double units = (ad_high * 256) + ad_low;
  
  double volts =  (units /4096)*3; 
  return volts;  
}
 
 
double getRoomTemperatureC()
{
  Wire.requestFrom(RoomTempI2CAddress,2);
  byte MSB = Wire.read();
  byte LSB = Wire.read();
 
  int TemperatureSum = ((MSB << 8) | LSB) >> 4;
  double celsius = TemperatureSum*0.0625;
  
  return celsius;
}
 
void SetRoomTemperataureResolutionBits(int ResolutionBits)
{
  if (ResolutionBits < 9 || ResolutionBits > 12) exit;
  Wire.beginTransmission(RoomTempI2CAddress);
  Wire.write(B00000001); //addresses the configuration register
  Wire.write((ResolutionBits-9) << 5); //writes the resolution bits
  Wire.endTransmission();
 
  Wire.beginTransmission(RoomTempI2CAddress); //resets to reading the temperature
  Wire.write((byte)0x00);
  Wire.endTransmission();
}




