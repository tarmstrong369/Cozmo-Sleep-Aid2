/* Adafruit LIS3DH ROS Keyboard Tap Code
   Base Tap Demo by Adafruit
   ROS Connection and keyboard tuning by: Rhian Preston
   Date: October 1, 2020
   license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.
   Source:

   Monitor keyboard use via vibration taps with the LIS3DH IMU using SPI

   Hardware setup:
   LIS3DH Breakout ----------- Teensyduino
   Vin ----------------------- Vcc
   SCL ----------------------- Digital 13 (B4)
   SDA ----------------------- Digital 11 (D6)
   SDO ----------------------- Digital 12 (D7)
   CS ------------------------ Digital 10 (C7)
   INT ----------------------- INT1 (6)
   GND ----------------------- GND

   XXX Push Button ----------- Teensyduino
    -------------------------- Digital 3
    -------------------------- GND

   NOTES:
*/

// #include <Wire.h>
// #include <SPI.h>
// #include <Adafruit_LIS3DH.h>
// #include <Adafruit_Sensor.h>

#include "Button.h"
#include "Statistic.h"

// // Used for software SPI
// #define LIS3DH_CLK 13
// #define LIS3DH_MISO 12
// #define LIS3DH_MOSI 11
// // Used for hardware & software SPI
// #define LIS3DH_CS 10

// // software SPI
// Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
// // hardware SPI
// //Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// // I2C
// //Adafruit_LIS3DH lis = Adafruit_LIS3DH();

#define SENSOR_PIN 2

#define SERIAL_DEBUG false
#define BUTTON_PIN 3

// Root mean square variables
//Statistic rms_stats;                // statistical set because I am lazy
//#define RMS_SAMPLE 500              // Change this to change how big of an initial sample set is taken;
//#define STD_SENSITIVITY 4           // Change this multiplier to change how sensitive to variations the check is (bigger value -> less sensitive)

// Push button variables
Button btn_break(BUTTON_PIN);
Button snsr_break(SENSOR_PIN);


void setup(void) {
  Serial.begin(115200);
  if (SERIAL_DEBUG) {
    while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

    Serial.print("Time: "); Serial.println(millis()); Serial.println("LIS3DH test!");
  }

//   if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
//     if (SERIAL_DEBUG) {
//       Serial.println("Couldnt start");
//     }
//     while (1) yield();
//   }
//   if (SERIAL_DEBUG) {
//     Serial.println("LIS3DH found!");

//     // lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!

//     Serial.print("Range = "); Serial.print(2 << lis.getRange());
//     Serial.println("G");

//     // lis.setDataRate(LIS3DH_DATARATE_50_HZ);
//     Serial.print("Data rate set to: ");
//     switch (lis.getDataRate()) {
//       case LIS3DH_DATARATE_1_HZ: Serial.println("1 Hz"); break;
//       case LIS3DH_DATARATE_10_HZ: Serial.println("10 Hz"); break;
//       case LIS3DH_DATARATE_25_HZ: Serial.println("25 Hz"); break;
//       case LIS3DH_DATARATE_50_HZ: Serial.println("50 Hz"); break;
//       case LIS3DH_DATARATE_100_HZ: Serial.println("100 Hz"); break;
//       case LIS3DH_DATARATE_200_HZ: Serial.println("200 Hz"); break;
//       case LIS3DH_DATARATE_400_HZ: Serial.println("400 Hz"); break;

//       case LIS3DH_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
//       case LIS3DH_DATARATE_LOWPOWER_5KHZ: Serial.println("5 Khz Low Power"); break;
//       case LIS3DH_DATARATE_LOWPOWER_1K6HZ: Serial.println("16 Khz Low Power"); break;
//     }
//   }

//   // Root mean square setup
//   rms_stats.clear(true); // Start clean and empty
}


void loop() {

//   /* Or....get a new sensor event, normalized */
//   sensors_event_t event;
//   if (lis.getEvent(&event)) {
//     // Get the root mean square of the acceleration to use for rolling vibration analysis
//     float rms = sqrt(sq(event.acceleration.x) + sq(event.acceleration.y) + sq(event.acceleration.z));
//     // If initial base rms not set, calculate starting average
//     if (rms_stats.count() <= RMS_SAMPLE) {
//       rms_stats.add(rms);
//     }
//     else {
//       float diff = abs(rms - rms_stats.average());
//       // If the RMS is outside the average bounds, probably a key press;
//       if (diff > (STD_SENSITIVITY * rms_stats.unbiased_stdev())) {
//         //str_keypress.data = "Key pressed";
//         //key_press.publish( &str_keypress );
//         Serial.println("key");
//         if (SERIAL_DEBUG) {
//           /* Display the results (acceleration is measured in m/s^2) */
//           Serial.print("Time: "); Serial.print(millis()); Serial.print("\tDiff: "); Serial.print(1000 * diff);
//           Serial.print("\tSTD: "); Serial.println(1000 * STD_SENSITIVITY * rms_stats.pop_stdev());
//           Serial.print("\tX: "); Serial.print(event.acceleration.x);
//           Serial.print(" \tY: "); Serial.print(event.acceleration.y);
//           Serial.print(" \tZ: "); Serial.print(event.acceleration.z);
//           Serial.println(" m/s^2 ");
//         }
//       }
//     }
//   }

  //


  // Button analysis and publish
  if (btn_break.isPressed()) {
    //str_btnpress.data = "Button pressed";
    //btn_press.publish( &str_btnpress );
    Serial.println("btn");
    if (SERIAL_DEBUG) {
      Serial.print("Time: "); Serial.print(millis()); Serial.println("\t\tButton Pressed!");
    }
  }

// Sensor analysis and publish
  if (snsr_break.isPressed()) {
    //str_btnpress.data = "Button pressed";
    //btn_press.publish( &str_btnpress );
    Serial.println("slp"); //sleeping
    if (SERIAL_DEBUG) {
      Serial.print("Time: "); Serial.print("millis()"); Serial.println("\t\tSensor Pressed!");
    }
  // else if (!snsr_break.isPressed()) {
  //   //str_btnpress.data = "Button pressed";
  //   //btn_press.publish( &str_btnpress );
  //   Serial.println("awk"); //awake
  //   if (SERIAL_DEBUG) {
  //     Serial.print("Time: "); Serial.print("millis()"); Serial.println("\t\tSensor Pressed!");
    // }
  }

}
