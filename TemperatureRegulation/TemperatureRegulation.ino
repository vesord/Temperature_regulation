#include <OneWire.h>
#define RELAY_PIN 6
#define POT_PIN A0
#define CLOCK_PIN 3
#define LATCH_PIN 4
#define DATA_PIN 5
#define SEG1_PIN 8
#define SEG2_PIN 9
#define SEG3_PIN 10
#define SEG4_PIN 11

OneWire ds(7); // Объект OneWire

int temperature = 0; // Глобальная переменная для хранения значение температуры с датчика DS18B20
long lastUpdateTime = 0; // Переменная для хранения времени последнего считывания с датчика
const int TEMP_UPDATE_TIME = 1000; // Определяем периодичность проверок
int lastTemperature = 9999;
int curTemperature;

byte digits_array[12] = {
    B00111111, B00000110, B01011011, B01001111, // 0 1 2 3
    B01100110, B01101101, B01111101, B00000111, // 4 5 6 7
    B01111111, B01101111, B00111001, B01100011 // 8 9 C о
  };

void setup(){
  Serial.begin(9600);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  
  pinMode(POT_PIN, INPUT);
  
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  
  pinMode(SEG1_PIN, OUTPUT);
  digitalWrite(SEG1_PIN, HIGH);
  pinMode(SEG2_PIN, OUTPUT);
  digitalWrite(SEG2_PIN, HIGH);
  pinMode(SEG3_PIN, OUTPUT);
  digitalWrite(SEG3_PIN, HIGH);
  pinMode(SEG4_PIN, OUTPUT);
  digitalWrite(SEG4_PIN, HIGH);
}

void loop(){

  int master;
  
  curTemperature = detectTemperature();
  if (curTemperature <= 0) 
  curTemperature = lastTemperature; 
  else 
  lastTemperature = curTemperature;
  Serial.println(curTemperature);

  master  = getMaster();
  //Serial.println(master);
  if (master > curTemperature) digitalWrite(RELAY_PIN, HIGH); else digitalWrite(RELAY_PIN, LOW);

  dispRefresh(master);
}

int getMaster(){
  return map(analogRead(POT_PIN), 0, 1024, 14, 33);
}

void dispRefresh(int master){
  ledOn(SEG1_PIN, master/10);
  ledOn(SEG2_PIN, master%10);
  ledOn(SEG3_PIN, 11);
  ledOn(SEG4_PIN, 10);
}

void ledOn(int seg, int digit){
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, digits_array[digit]); 
  digitalWrite(LATCH_PIN, HIGH);

  digitalWrite(seg, LOW);
  delay(5);
  digitalWrite(seg, HIGH);
}

int detectTemperature() {
  if (millis() - lastUpdateTime > TEMP_UPDATE_TIME)
  {
  lastUpdateTime = millis();
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;
  
  if ( !ds.search(addr)) {
    //Serial.println("No more addresses.");
    //Serial.println();
    ds.reset_search();
    delay(50);
    return -1;
  }
  
  /*Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }*/
  //Serial.println();
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      //Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      //Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      //Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      //Serial.println("Device is not a DS18x20 family device.");
      return -1;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  //Serial.print("  Data = ");
  //Serial.print(present, HEX);
  //Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    //Serial.print(data[i], HEX);
    //Serial.print(" ");
  }
  //Serial.print(" CRC=");
  //Serial.print(OneWire::crc8(data, 8), HEX);
  //Serial.println();

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
  //fahrenheit = celsius * 1.8 + 32.0;
  //Serial.print("  Temperature = ");
  //Serial.print(celsius);
  //Serial.print(" Celsius, ");
  //Serial.print(fahrenheit);
  //Serial.println(" Fahrenheit");
  return celsius;
  }
  else
  return -1;
}
