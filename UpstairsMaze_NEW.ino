// upstairs code for keypad and lcd
#include <MPU6050_tockn.h>
#include <Keypad.h>
#include <LiquidCrystal.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

const int rs = 4, en = 16, d4 = 5, d5 = 18, d6 = 19, d7 = 0; // change rs and en pins on bot !!!! Note: GPIO0 (d7) seems a bit sus
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//3x4 Matrix key pad
const byte ROWS = 4; // Four rows
const byte COLS = 3; // three columns

  // Define the Keymap
  char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
  };

 byte rowPins[ROWS] = {12, 14, 27, 26};
 byte colPins[COLS] = {25, 33, 32};

 Keypad kpd = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

 const int slaveAddress = 8;
// 5 8 6 6 9 2  8
void setup()  
{
    Wire.begin();
    Serial.begin(9600);
    lcd.begin(16, 2);  
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("EEEBot: Maze"); // Try and display this for 1 second then display EEEBot calibrating
    mpu6050.begin(); // Initialize the MPU6050
    mpu6050.calcGyroOffsets(true); // Make sure the sensor is stable and not moving before running this
}

void loop() {
   mpu6050.update(); // Update sensor data
   int pitch = mpu6050.getAngleX(); // Get the pitch angle
   Serial.println(pitch);

   char key = kpd.getKey();
   if (!key) {
     key = 0xFF; // Use a placeholder for no key press
   }

   Wire.beginTransmission(slaveAddress); // Transmit to device with address 8
   Wire.write((pitch >> 8) & 0xFF); // High byte of pitch
   Wire.write(pitch & 0xFF);       // Low byte of pitch
   Wire.write(key);                // Send key or placeholder
   Wire.endTransmission();

   // Debugging output
   if (key != 0xFF) {
     Serial.println(key);
     lcd.setCursor(9, 1);
     lcd.print("KEY= ");
     lcd.print(key);
   }
   delay(100); // Delay to prevent overwhelming the I2C bus
}

