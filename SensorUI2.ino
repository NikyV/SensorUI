#include <Wire.h> // Arduino library for I2C
#include <LiquidCrystal_I2C.h> //I2C interface
//#include <LiquidCrystal.h> // Library for LCD
#include <Keypad.h>
#define LED 12
#define BUZZER 11



// -----------------------------------------------------------------------------
// Sensor specific settings, adjust if needed:
// -----------------------------------------------------------------------------

const int ADDRESS = 0x08; // Sensor I2C Address
const float SCALE_FACTOR_FLOW = 10000; // Scale Factor for flow rate measurement
const float SCALE_FACTOR_TEMP = 200.0; // Scale Factor for temperature measurement
const char *UNIT_FLOW = " ml/min"; //physical unit of the flow rate measurement
const char *UNIT_TEMP = " deg C"; //physical unit of the temperature measurement


//LiquidCrystal lcd(A0,A1,A2,A3,A4,12);
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2); // Change to (0x27,20,4) for 20x4 LCD.

const byte ROWS = 4; 
const byte COLS = 4; 

char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'.', '0', '#', 'D'}
};

byte rowPins[ROWS] = {9, 8, 7, 6}; 
byte colPins[COLS] = {5, 4, 3, 2}; 

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 

// things to declare
bool isInput = false;
float val = 0;
bool alarmOn = false;

// things to declare for integration
int old_time;
int new_time;
float time_diff;
float vol;
float total_vol= 0.0;


void setup()
{
 //lcd.begin(16, 2); 
 lcd.init();
 lcd.backlight();
 pinMode(LED, OUTPUT);
 pinMode(BUZZER, OUTPUT);
 //pinMode(resetPin,INPUT);
 //digitalWrite(resetPin, LOW);

   int ret;

  Serial.begin(9600); // initialize serial communication
  Wire.begin();       // join i2c bus (address optional for master)


  do {
    // Soft reset the sensor
    Wire.beginTransmission(0x00);
    Wire.write(0x06);
    ret = Wire.endTransmission();
    Serial.println(ret);
    if (ret != 0) {
      Serial.println("Error while sending soft reset command, retrying...");
      delay(500); // wait long enough for chip reset to complete
    }
  } while (ret != 0);

  delay(100); // wait long enough for chip reset to complete
}


void loop()
{
    int ret;
  uint16_t aux_value;
  uint16_t sensor_flow_value;
  uint16_t sensor_temp_value;
  int16_t signed_flow_value;
  int16_t signed_temp_value;
  float scaled_flow_value;
  float scaled_temp_value;
  byte aux_crc;
  byte sensor_flow_crc;
  byte sensor_temp_crc;


  // To perform a measurement, first send 0x3608 to switch to continuous
  // measurement mode (H20 calibration), then read 3x (2 bytes + 1 CRC byte) from the sensor.
  // To perform a IPA based measurement, send 0x3615 instead.
  // Check datasheet for available measurement commands.
  Wire.beginTransmission(ADDRESS);
  Wire.write(0x36);
  Wire.write(0x08);
  ret = Wire.endTransmission();
  if (ret != 0) {
    Serial.println("Error during write measurement mode command");

  } else {
    //delay(1000);

    for(int i = 0; i < 10; ++i) {
      delay(1000);
      Wire.requestFrom(ADDRESS, 9);
      if (Wire.available() < 9) {
        Serial.println("Error while reading flow measurement");
        continue;
      }

      sensor_flow_value  = Wire.read() << 8; // read the MSB from the sensor
      sensor_flow_value |= Wire.read();      // read the LSB from the sensor
      sensor_flow_crc    = Wire.read();
      sensor_temp_value  = Wire.read() << 8; // read the MSB from the sensor
      sensor_temp_value |= Wire.read();      // read the LSB from the sensor
      sensor_temp_crc    = Wire.read();
      aux_value          = Wire.read() << 8; // read the MSB from the sensor
      aux_value         |= Wire.read();      // read the LSB from the sensor
      aux_crc            = Wire.read();

      //Serial.print("Flow value from Sensor: ");
      //Serial.print(sensor_flow_value);

      signed_flow_value = (int16_t) sensor_flow_value;
      //Serial.print(", signed value: ");
      //Serial.print(signed_flow_value);

      scaled_flow_value = ((float) signed_flow_value) / SCALE_FACTOR_FLOW;
      //Serial.print(", scaled value: ");
      Serial.print(scaled_flow_value);
      //Serial.print(UNIT_FLOW);

      //Serial.print(", Temp value from Sensor: ");
      //Serial.print(sensor_temp_value);

      signed_temp_value = (int16_t) sensor_temp_value;
      //Serial.print(", signed value: ");
      //Serial.print(signed_temp_value);

      scaled_temp_value = ((float) signed_temp_value) / SCALE_FACTOR_TEMP;
      //Serial.print(", scaled value: ");
      //Serial.print(scaled_temp_value);
      //Serial.print(UNIT_TEMP);

      Serial.println("");
  
  float sensorValue = scaled_flow_value; // simulated sensor value
  char key = customKeypad.getKey(); // keypad input
  Serial.println(key);


switch (key)
  {
    case NO_KEY:
    lcd.setCursor(0,0);
    lcd.print(sensorValue,3);
    lcd.setCursor(6,0);
    lcd.print("mL/min");
    new_time = millis();
    time_diff = (new_time - old_time)*(1.0/60000);
    old_time = new_time;
    vol = (sensorValue)*time_diff;
    total_vol = total_vol + vol;
    lcd.setCursor(0,1);
    lcd.print("Total Vol=");
    lcd.print(total_vol);
    lcd.print("mL");
    break;
    
    case '#':
    isInput = false;
    val = 0;
    val = inputThreshold();
    delay(300);
    lcd.clear();
    break;

    case 'D':
    isInput = false;
    lcd.clear();
    while (isInput == false){
      //lcd.clear();
        lcd.setCursor(0,0);
      lcd.print("Threshold set to ");
      lcd.setCursor(0,1);
      lcd.print(val);
      lcd.print("mL");
        
        char key = customKeypad.getKey();
        if (key == 'D') {
          lcd.clear();  
            break;
        }
      
    }
    case 'B':
    isInput=false;
    //char key = customKeypad.getKey();
    if(key=='B') {
      resetVolume();
      break;
    }
  
  }
  
  if (total_vol > val && isInput == true){
    lcd.clear();
   alarmOn = true;

  }

  while(alarmOn == true)
  {
   //char key = customKeypad.getKey(); // keypad input
   lcd.setCursor(2,0); //sets column location in first row 
   lcd.print("check patient");//shown on first row
   lcd.setCursor(4,1);//sets column location in second row 
   lcd.print("and EVD"); //shown on second row 
   digitalWrite(LED, HIGH);
   tone(BUZZER, 500, 300);
   //delay(1000);
   //digitalWrite(LED, LOW);
    
    char key = customKeypad.getKey(); // keypad input
    if(key == 'C'){
      alarmOn = false;
      isInput = false;
      lcd.clear();
      lcd.setCursor(0,1);
      digitalWrite(LED, LOW);
      digitalWrite(BUZZER, LOW);
      break;
   }
  }
      
    }
    // To stop the continuous measurement, first send 0x3FF9.
    Wire.beginTransmission(ADDRESS);
    Wire.write(0x3F);
    Wire.write(0xF9);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error during write measurement mode command");
    }
  }

  
  //sensorValue = analogRead(5);

  
  
  //------------------------------------------------
  // press # to enter input threshold function
  // press A to exit function
  // "*" is used as "."
  // -----------------------------------------------
  
  
}


void resetVolume(){
  total_vol= 0.0;
}




float inputThreshold(){
  char enteredFlow[6] = {'\0'};
  int count=0;
  //float setThresh = 0;
  float setThresh = 0; // threshold value to be returned
  lcd.clear();
  
  while (isInput == false){
    lcd.setCursor(0,0);
    lcd.print("threshold:");
    lcd.setCursor(5,1);
    lcd.print("mL");    
    
    char key = customKeypad.getKey();
    lcd.setCursor(0,1);
    //if (key != NO_KEY)  //NEED TO FOOLPROOF THIS
    if(key!=NO_KEY && key != 'B' && key!= 'C' && key!= 'D' && key != '#') 
    {
      enteredFlow[count]=key;
      count++;
      setThresh = atof(enteredFlow);
      lcd.print(setThresh, 2);
    }
    if (key == 'A')
    {
      isInput = true;
    }
    
  }
  
  if (setThresh == 0)
  {
    lcd.clear();
    lcd.print("input error");
    delay(2000);
    isInput = false;
  }
  return setThresh;

      
}
