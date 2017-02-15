#include <Wire.h>
#include <Arduino.h>
#include "TempConfig.h";
#include "IMUConfig.h";
#include "Max30100Config.h";
#include "BLEConfig.h";
#include "TFTConfig.h";

// Callback (registered below) fired when a pulse is detected
void onBeatDetected()
{
  Serial.println("Beat!");
}

//void testdrawtext(char *text, uint16_t color) {
//  tft.setCursor(0, 0);
//  tft.setTextColor(color);
//  tft.setTextWrap(true);
//  tft.print(text);
//}
void TFT_Setup()
{
  //TFT Setup
  //tft.fillScreen(ST7735_WHITE);
  tft.setTextWrap(false);
  tft.setTextColor(ST7735_MAGENTA);
  tft.setTextSize(0.2);
  bmpDraw("capstone.bmp", 0, 0);
  delay(500);
}

void setup() {
  Serial.begin(115200);
  //IMU
  //Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  //  if (!bno.begin())
  //  {
  //    /* There was a problem detecting the BNO055 ... check your connections */
  //    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  //    testdrawtext("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!", ST7735_RED);
  //    while (1);
  //  }
  //
  //  delay(1000);
  //
  /* Display the current temperature */
  //  int8_t temp = bno.getTemp();
  //  Serial.print("Current Temperature: ");
  //  Serial.print(temp);
  //  Serial.println(" C");
  //  Serial.println("");

  bno.setExtCrystalUse(true);

  //Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  ///////TFT
  tft.initR(INITR_144GREENTAB);

  //  Serial.print("Initializing SD card...");
  //  if (!SD.begin(SD_CS)) {
  //    Serial.println("failed!");
  //    return;
  //  }
  TFT_Setup();

  ///Timer
  uint16_t time = millis();
  time = millis() - time;

  //  Serial.println(time, DEC);
  delay(500);

  //  Serial.println("Initializing MAX30100");
  // Initialize the PulseOximeter instance and register a beat-detected callback
  pox.begin();
    pox.setOnBeatDetectedCallback(onBeatDetected);
  ble_init();

}
//Initiate BLE
void ble_init() {
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  //  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    //    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  //  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  //  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  //  Serial.println(F("Then Enter characters to send to Bluefruit"));
  //  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  //while (! ble.isConnected()) {
  //  delay(500);
  //}

  //  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    //    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set module to DATA mode
  //  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  //  Serial.println(F("******************************"));
}


//#define BUFFPIXEL 20//

void bmpDraw(char *filename, uint8_t x, uint8_t y) {

  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3 * 20];//BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();

  if ((x >= tft.width()) || (y >= tft.height())) return;

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    return;
  }

  // Parse BMP header
  if (read16(bmpFile) == 0x4D42) { // BMP signature
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    // Read DIB header
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if (read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      if ((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if (bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if ((x + w - 1) >= tft.width())  w = tft.width()  - x;
        if ((y + h - 1) >= tft.height()) h = tft.height() - y;

        // Set TFT address window to clipped image bounds
        tft.setAddrWindow(x, y, x + w - 1, y + h - 1);

        for (row = 0; row < h; row++) { // For each scanline...

          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if (flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if (bmpFile.position() != pos) { // Need seek?
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }

          for (col = 0; col < w; col++) { // For each pixel...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format, push to display
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            tft.pushColor(tft.Color565(r, g, b));
          } // end pixel
        } // end scanline
      } // end goodBmp
    }
  }

  bmpFile.close();
  if (!goodBmp) Serial.println("BMP format not recognized.");
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}

/////////////////////////////
//////////////BLE
void ble_sending_phone() {
  if (Serial.available())
  {
    char n, inputs[BUFSIZE + 1];
    n = Serial.readBytes(inputs, BUFSIZE);
    inputs[n] = 0;
    // Send characters to Bluefruit
    //    Serial.print("Sending: ");
    //    Serial.println(inputs);

    // Send input data to host via Bluefruit
    ble.print(inputs);

  }

  // Echo received data
  while ( ble.available() )
  {
    char c = ble.read();
    //    Serial.print(c);

    // Hex output too, helps w/debugging!
    //Serial.print(" [0x");
    //if (c <= 0xF) Serial.print(F("0"));
    // Serial.print(c);//, HEX);
    //Serial.print("] ");
    //    tft.print(c);
  }
}

////////////////////////
////////////External Temp.
void ExtTemp_Calc_Print()
{
  tft.setCursor(90, 112);
  tft.print(bno.getTemp(), 2);

}
/////////////////////////////
////////////Temp Sense
void TempSense_Calc_Print()
{
  //Calculation
  int sensorValue = analogRead(tempPin);
  double mvolts = (sensorValue * (3.3 / 1023.0)) * 1000;
  //  Serial.print(mvolts);
  double Temp_cel = (10.888 - sqrt((-10.888) * (-10.888) + 4 * 0.00347 * (1777.3 - mvolts))) / (2 * (-0.00347)) + 30;
  //  float Temp_farh = Temp_cel * (9 / 5) + 32;
  //Serial Print
  //  Serial.print(" TEMPRATURE = ");
  //  Serial.print(Temp_cel);
  //  Serial.print("*C / ");
  //  Serial.print(Temp_farh);
  //  Serial.println("*F");
  //TFT Print
  //  tft.setTextColor(ST7735_WHITE);
  //  tft.print("Body Temp: ");
  //  tft.setTextColor(ST7735_RED);
  //  tft.print(Temp_cel);
  //  tft.setTextColor(ST7735_WHITE);
  //  tft.println(" C / ");
  //  tft.setTextColor(ST7735_RED);
  //  tft.print(Temp_farh);
  //  tft.setTextColor(ST7735_WHITE);
  //  tft.println(" F");
  tft.setCursor(90, 48);
  tft.print(Temp_cel, 2);

  //print BLE
  if (ble.isConnected())
  {
    ble.print(" TEMPRATURE = ");
    ble.print(Temp_cel);
    ble.print("*C / ");
    //    ble.print(Temp_farh);
    //    ble.println("*F");
  }
}

/////////////////////
///////////////HR/SPO2
void HR_SPO2_Calc_Print()
{
  pox.update();

  // Asynchronously dump heart rate and oxidation levels to the serial
  // For both, a value of 0 means "invalid"
  //    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
  /////////////////////////////////Heart Rate/////////////////////////////////
  //TFT
  //  tft.setTextColor(ST7735_WHITE);
  //  tft.print("Heart rate: ");
  //  tft.setTextColor(ST7735_RED);
  //  tft.print(pox.getHeartRate(), 2);
  //  tft.setTextColor(ST7735_WHITE);
  //  tft.println(" bpm");
  tft.setCursor(90, 16);
  tft.print(pox.getHeartRate(), 2);

  //Serial
  //  Serial.print("Heart rate:");
  //  Serial.print(pox.getHeartRate());
  //  Serial.print("bpm");
  //BLE
  if (ble.isConnected())
  {
    ble.print("Heart rate:");
    ble.print(pox.getHeartRate());
    ble.print("bpm");
  }
  /////////////////////////////////Blood Oxidation/////////////////////////////////
  //TFT
  //  tft.print(" / SpO2: ");
  //  tft.setTextColor(ST7735_RED);
  //  tft.print(pox.getSpO2());
  //  tft.setTextColor(ST7735_WHITE);
  //  tft.println(" %");
  tft.setCursor(90, 80);
  tft.print(pox.getSpO2());
  //Serial
//    Serial.print(" SpO2:");
//    Serial.print(pox.getSpO2());
  //  Serial.print("%");
  //BLE
  if (ble.isConnected())
  {
    ble.print(" SpO2:");
    ble.print(pox.getSpO2());
    ble.print("%");
  }
  /////////////////////////////////Temperature/////////////////////////////////
  //TFT
  //  tft.print("Temperature: ");
  //  tft.setTextColor(ST7735_RED);
  //  tft.print(pox.getTemperature(), 2);
  //  tft.setTextColor(ST7735_WHITE);
  //  tft.println(" C");
  //Serial
  //  Serial.print(" / temp:");
  //  Serial.print(pox.getTemperature());
  //  Serial.println("C");
  //BLE
  if (ble.isConnected())
  {
    ble.print(" / temp:");
    ble.print(pox.getTemperature());
    ble.println("C");
  }
  //        tsLastReport = millis();
  //    }
}

void IMU_Calc_Print()
{
  //IMU
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  //  Serial.print("[");
  //  Serial.print(euler.x());
  //  Serial.print(", ");
  //  Serial.print(euler.y());
  //  Serial.print(", ");
  //  Serial.print(euler.z());
  //  Serial.print("]\t\t");
  //BLE
  if (ble.isConnected())
  {
    ble.print("[");
    ble.print(euler.x());
    ble.print(", ");
    ble.print(euler.y());
    ble.print(", ");
    ble.print(euler.z());
    ble.print("]\t\t");
  }

  /*
    // Quaternion data
    imu::Quaternion quat = bno.getQuat();
    Serial.print("qW: ");
    Serial.print(quat.w(), 4);
    Serial.print(" qX: ");
    Serial.print(quat.y(), 4);
    Serial.print(" qY: ");
    Serial.print(quat.x(), 4);
    Serial.print(" qZ: ");
    Serial.print(quat.z(), 4);
    Serial.print("\t\t");
  */

  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  //  Serial.print("CALIBRATION: Sys=");
  //  Serial.print(system, DEC);
  //  Serial.print(" Gyro=");
  //  Serial.print(gyro, DEC);
  //  Serial.print(" Accel=");
  //  Serial.print(accel, DEC);
  //  Serial.print(" Mag=");
  //  Serial.println(mag, DEC);
  //BLE
  if (ble.isConnected())
  {
    ble.print("CALIBRATION: Sys=");
    ble.print(system, DEC);
    ble.print(" Gyro=");
    ble.print(gyro, DEC);
    ble.print(" Accel=");
    ble.print(accel, DEC);
    ble.print(" Mag=");
    ble.println(mag, DEC);
  }
  //delay(BNO055_SAMPLERATE_DELAY_MS);
}

void loop() {
  pox.update();
  if (millis() - tsLastReport > 1000) {
    tft.fillRoundRect(90, 0, 38 , 128, 0, ST7735_WHITE);
    bmpDraw("capstone.bmp", 0, 0);
    //Heart Rate and Blood Oxidation
    HR_SPO2_Calc_Print();
    //Temperature Sensing IC
    TempSense_Calc_Print();
    //External Temperature
    //ExtTemp_Calc_Print();
    //Inertial Measurement Unit
    IMU_Calc_Print();
    //BLE Send
    ble_sending_phone();
    tsLastReport = millis();
  }
}
