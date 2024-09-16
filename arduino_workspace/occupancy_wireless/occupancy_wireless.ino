/*********************************************************************
 Derived from Adafruit nRF52 based Bluefruit LE modules example

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <bluefruit.h>


const int SENSOR_PIN = 5;       // number of sensor input pin

const char ID[] = "1";          // id tag for this board
uint8_t mac[6];                 // array that will hold the MAC address
char bluetooth_name[50] = {0};  // buffer array for the bluetooth advertised name

// State variables
String message;
int data;

// BLE Service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble


void setup()
{
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);

  // Extract address and name accordingly
  // error with library: mac address is backwards
  Bluefruit.Gap.getAddr(mac);
  sprintf(bluetooth_name, "SHARE-%s-%02x:%02x:%02x:%02x:%02x:%02x", 
    ID, mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
  
  Bluefruit.setName(bluetooth_name);

  // Configure and Start Device Information Service
  bledis.setManufacturer("Sparkfun");
  bledis.setModel("nrf52840 mini");
  bledis.begin();
  
  // Configure and Start BLE Uart Service
  bleuart.begin();
  // Set up and start advertising
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);
  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}


void loop()
{
  // Poll data from chair sensor
  data = analogRead(SENSOR_PIN);

  // Fill a buffer with the data and send it
  message = String(millis()) + " " + String(data);
  uint8_t buf[64];
  message.getBytes(buf, sizeof(buf));
  bleuart.write(buf, message.length());
  // Also print the message for sanity checking
  Serial.println(message);
}
