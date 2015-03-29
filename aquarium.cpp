
// include the library code:
#include <LiquidCrystal.h>
#include <Time.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>
#include <OneWire.h>

/* ******** Ethernet Card Settings ******** */
// Set this to your Ethernet Card Mac Address
byte mac[] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF }; 

/* ******** NTP Server Settings ******** */
/* us.pool.ntp.org NTP server 
   (Set to your time server of choice) */
IPAddress timeServer(132, 163, 4, 101);

/* Set this to the offset (in seconds) to your local time
   This example is GMT + 3 */
const long timeZoneOffset = 10800L;   

/* Syncs to NTP server every minute */
unsigned int ntpSyncTime = 60;         

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

// ThingSpeak Setup
char thingSpeakAddress[] = "api.thingspeak.com";
const int updateThingSpeakInterval = 60 * 1000;      // Time interval in milliseconds to update ThingSpeak (number of seconds * 1000 = interval)
String myThingSpeakKey = "POJ06PXJKR76NB39";  
long lastConnectionTime = 0; 
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
const int DESIRED_BRIGHTNESS = (255 * 5) / 10;

float temperature = 0;

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
    delay(2000);
    lcd.clear();
}

void displaySystem(float temp, int co2Status){

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
  lcd.print(dayShortStr(weekday()));
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
    delay(500);
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
  
  delay(1500);     // maybe 750ms is enough, maybe not
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
  //Serial.print("  Temperature = ");
  //Serial.print(celsius);
  //Serial.print(" Celsius, ");
  
  return celsius;
}

//Heater Control 
void heaterControl(float temp){
  
  if(temp > 0 && temp < 25){ //>0 to avoid -999.00*C error   
    digitalWrite(HEATER, HIGH);
    //Serial.println("Heating..."); 
  } else if(temp > 25 && temp < 50){ //<0 to avoid -999.00*C error
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

void updateThingSpeak(String tsData)
{
  if (client.connect(thingSpeakAddress, 80)){

    client.print("POST /update HTTP/1.1\n");
    client.print("Host: api.thingspeak.com\n");
    client.print("Connection: close\n");
    client.print("X-THINGSPEAKAPIKEY: "+myThingSpeakKey+"\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Length: ");
    client.print(tsData.length());
    client.print("\n\n");
    client.print(tsData);
    
    lastConnectionTime = millis();
    if (client.connected())
        {
          Serial.println("Connecting to ThingSpeak...");
          Serial.println();
          failedCounter = 0;
        }
    else{
          failedCounter++;
          Serial.println("Connection to ThingSpeak failed ("+String(failedCounter, DEC)+")");   
          Serial.println();
    }
  }
  else{

    failedCounter++;
    Serial.println("Connection to ThingSpeak Failed ("+String(failedCounter, DEC)+")");   
    Serial.println();
    lastConnectionTime = millis(); 
  }
}

void setup() {

  // Setting welcome screen
  welcomeLcd(); 

  getTimeNTP();

  // set system defaults
  setDefaults();

  Serial.begin(9600);
    
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

  delay(1000);
  float pass = checkWaterTemp();
  if( pass > 1)
    temperature = pass;

}

void controlMoonLeds(unsigned long myTime){

  byte first = brightness(myTime, 22*HOUR+45*MINUTE, 23*HOUR+45*MINUTE, "moon");

  if(first == 0){
    digitalWrite(MOONLIGHTS, LOW);
  } else{
      digitalWrite(MOONLIGHTS, HIGH);
  }
}

int controlLeds(unsigned long myTime){

  int openFan = 0;

  byte first = brightness(myTime, 10*HOUR+30*MINUTE, 13*HOUR+45*MINUTE, "led");
  byte second = brightness(myTime, 18*HOUR, 22*HOUR+45*MINUTE, "led");
  //revert the pwm value because of the pwm curcuit
  second = map(second, PWM_MIN, PWM_MAX, PWM_MAX, DESIRED_BRIGHTNESS);
  first = map(first, PWM_MIN, PWM_MAX, PWM_MAX, DESIRED_BRIGHTNESS);

  if(first == PWM_MIN || second == PWM_MIN){
    digitalWrite(LIGHTS, HIGH);
    analogWrite(LIGHTS_PWM, DESIRED_BRIGHTNESS);
    openFan = 1;

  } else if(first == PWM_MAX && second == PWM_MAX){
      digitalWrite(LIGHTS, LOW);
      openFan = 0;

  } else{
      digitalWrite(LIGHTS, HIGH);
      if(first < second){
        analogWrite(LIGHTS_PWM, first);
      }
      else if(second < first){
        analogWrite(LIGHTS_PWM, second);
      }
      openFan = 1;
  }
  
  return openFan;
}

int controlCO2(unsigned long myTime){

  int openCo2 = 0;

  if ((myTime >= 9*HOUR+30*MINUTE  && myTime <= 13*HOUR+30*MINUTE) || (myTime >= 17*HOUR && myTime <= 22*HOUR+30*MINUTE) ){
    //open co2
    digitalWrite(CO2, HIGH);
    openCo2 = 1;
  } else{
      //close co2
      digitalWrite(CO2, LOW);
  }

  return openCo2;
}

byte brightness(unsigned long lastDaySeconds, unsigned long fadeUpStart, unsigned long fadeDownStart, String myLed)
{
  
  if (lastDaySeconds >= fadeUpStart + MINUTE*15 && lastDaySeconds <= fadeDownStart){
    return TARGET_BRIGHTNESS;
  }

  // Dawn:  fade up the light
  if (lastDaySeconds >= fadeUpStart && lastDaySeconds <= fadeUpStart + 15*MINUTE)  // Fading up
  {
    unsigned long seconds = lastDaySeconds - fadeUpStart;  // Number of seconds into the fade time 
    return TARGET_BRIGHTNESS * seconds / (MINUTE*15);  // Fade up based on portion of interval completed.
  }

  // Evening: Fade down the light
  if (lastDaySeconds >= fadeDownStart && lastDaySeconds <= fadeDownStart + 15*MINUTE)  // Fading down
  {
    unsigned long seconds = (fadeDownStart + (MINUTE*15)) - lastDaySeconds;  // Number of seconds remaining in the fade time 
    return TARGET_BRIGHTNESS * seconds / (MINUTE*15);  // Fade down based on portion of interval left.
  }

  // The remaining times are night and the lights is off
  return 0;  // Shouldn't get here
}

void loop() {

  unsigned long today = hour() * HOUR + minute() * MINUTE + second();

  //check the water tempature every 5 secs.
  if (now() % 5 == 0){
    float temp = checkWaterTemp();
    if(temp > 0 && temp < 40)
      temperature = temp;
  }

  heaterControl(temperature);
  int co2Check = controlCO2(today); 

  int fanCheck = controlLeds(today);
  fanControl(fanCheck);
  controlMoonLeds(today);

  // Update the time via NTP server as often as the time you set at the top
  if(now()-ntpLastUpdate > ntpSyncTime) {
    int trys=0;
    while(!getTimeAndDate() && trys<10){
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
    displaySystem(temperature, co2Check);  
  }

  // Print Update Response to Serial Monitor
  if (client.available()){
    char c = client.read();
    Serial.print(c);
  }

  // Disconnect from ThingSpeak
  if (!client.connected() && lastConnected){
      Serial.println("...disconnected");
      Serial.println();
      client.stop();
    }

  // Update ThingSpeak channel with my aquarium features
  if(!client.connected() && (millis() - lastConnectionTime > updateThingSpeakInterval)){

      String waterTemp = String(temperature, DEC);
      String daylights = String(digitalRead(LIGHTS), DEC);
      String moonlights = String(digitalRead(MOONLIGHTS), DEC);
      String co2String = String(digitalRead(CO2), DEC);
      String heaterString = String(digitalRead(HEATER), DEC);
      //String fanString = String(digitalRead(FANS), DEC);

      Serial.print("waterTemp:");
      Serial.println(temperature);
      Serial.print("daylights:");
      Serial.println(daylights);
      Serial.print("moonlights:");
      Serial.println(moonlights);
      Serial.print("co2String:");
      Serial.println(co2String);
      Serial.print("heaterString:");
      Serial.println(heaterString);

      //HTTP POST the values
      //updateThingSpeak("field1="+waterTemp+"&field2="+co2String"+&field3="+daylights+"&field4="+moonlights+"&field5="+heaterString);
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
int getTimeAndDate() {
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

void getTimeNTP(){

  // Ethernet shield and NTP setup
   int i = 0;
   int DHCP = 0;
   DHCP = Ethernet.begin(mac);
   //Try to get dhcp settings 30 times before giving up
   while( DHCP == 0 && i < 30){
     delay(1000);
     DHCP = Ethernet.begin(mac);
     i++;
   }
   if(!DHCP){
    Serial.println("DHCP FAILED");
    resetFunc();  //call reset
    // for(;;); //Infinite loop because DHCP Failed
   }
   Serial.println("DHCP Success");
   
   //Try to get the date and time
   int trys=0;
   while(!getTimeAndDate() && trys<10) {
     trys++;
   }

}
