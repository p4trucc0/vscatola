/*
 * Andrea Patrucco, 22/09/2020
 * Code for Arduino Uno - based GPS + IMU Serial and SD card logger
 * 
 * Requires: MPU6050 imu module, GT-U7 GPS, HW125 SD-Card SPI interface
 * 
 * PIN information:
 * Digital 3 -- GT-U7 TxD
 * Digital 4 -- GT-U7 RxD
 * +3,3V -- MPU6050 VCC & GT-U7 VCC
 * GND -- MPU6050 GND & GT-U7 GND & HW125 GND
 * Analog 5 -- MPU6050 SCL
 * Analog 4 -- MPU6050 SDA
 * 5V -- HW125 VCC
 * Digital 13 - HW125 SCK
 * Digital 12 - HW125 MISO
 * Digital 11 - HW125 MOSI
 * Digital 10 - HW125 CS
 * 
 * Pretty obviously includes lots of material mainly stolen from Stack Overflow,
 * Arduino website and tutorials around the net.
*/
#include <Wire.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include "SdFat.h"

// The first for SD SPI communication, the second to increase buffer for NMEA strings.
#define SPI_SPEED SD_SCK_MHZ(50)
#define _SS_MAX_RX_BUFF (128)

// Build variants
#define LOG_ON_SERIAL           // Turn on Serial printing
#define LOG_ON_SD               // Turn on logs on SD card
//#define DEBUG_SERIAL            // Add debug info (bytes available after GPS read) on Serial
#define PRIORITY_GPS            // Read GPS transmission before checking IMU timing. Needed for fast GPS on SD.
#define MANUAL_FLUSH_SIZE (4)   // Close and re-open SD file each N logged lines. Allows lower writing times.
#define SAMPLING_DT_MS (50.0)   // milliseconds between two consecutive IMU readings.
//#define GPS_10_HZ             // GPS measuring/publishing interval. Choose one between this and the following.
#define GPS_5_HZ

// Functional constants
#define BAUD_RATE (115200)      // Serial output baud
#define SW_BAUD_RATE (38400)    // SW Serial (GPS input)
#define TRY_AUTOCONFIG          // Send to the uBlox GPS config messages.
#define CSTRING_LEN (100)       // Pre-allocated space for GPS strings (C)

// Offsets for MPU6050. Replace with your sample's values.
#define OFFSET_AX (2265)
#define OFFSET_AY (-248)
#define OFFSET_AZ (251)
#define OFFSET_GX (-235)
#define OFFSET_GY (-5900)
#define OFFSET_GZ (590)

// MPU6050-related global variables
const int MPU = 0x68u;                  // I2C MPU6050 address
int16_t ax, ay, az, gx, gy, gz, temp;   
float time_true, time_ms;               

// GT-U7 related global variables
bool new_gps_data;                      
char c2read;
int i_ca;
int serial_bytes_available;             // for debug purposes
char ca2read[CSTRING_LEN];
char ca2send[CSTRING_LEN];
bool new_gps_trasm;

// SD handling global variables.
File myFile;
char c_fname[12] = "LOGAAAA.txt";
char c_string_number[5];
String fname;
String string_number;
uint16_t logno;
bool keep_searching_file;
#ifdef MANUAL_FLUSH_SIZE
uint8_t strings_written;
#endif
SdFat sd;

// UBX-CFG messages for GT-U7 GPS initial configuration.
const byte bac_set_5Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6a}; // LEN 14
const byte bac_set_10Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12}; // LEN 14
const byte bac_disable_VTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47}; // LEN 16
const byte bac_disable_GGA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24}; // LEN 16
const byte bac_disable_GLL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B}; // LEN 16
const byte bac_disable_GSA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32}; // LEN 16
const byte bac_disable_GSV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39}; // LEN 16
const byte bac_highbaud[] = {0xb5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xd0, 0x08, 0x00, 0x00, 0x00, 0xc2, 0x01, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc4, 0x96, 0xb5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x01, 0x08, 0x22}; // LEN37
const byte bac_baud38400[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x93, 0x90}; // LEN 28

SoftwareSerial gpsSerial(3, 4); // RX, TX (to match GT-U7 TX and RX respectively)


// Function prototypes
void sd_print_gps();
void sd_print_acc();
void serial_print_gps();
void serial_print_acc();


void setup()
{
  Wire.begin();                         // Initialize I2C as Master
  Wire.beginTransmission(MPU);          // Begin transmission to device at address "MPU"
  Wire.write(0x6B);                     // Write to PWR_MGMT_1 register...
  Wire.write(0);                        // ...value 0
  Wire.endTransmission(true);           // true to send a "stop" signal to I2C releasing the bus.
  Wire.beginTransmission(MPU);      
  Wire.write(0x1C);                     // range register
  Wire.write(0b00010000);               // range is +/-8g for all acc axes and +/-205°/s for gyros
  Wire.endTransmission(true); 
  Serial.begin(BAUD_RATE);
  time_ms = millis();
  gpsSerial.begin(9600);                // Emulated serial. Start at 9600 as GT-U7 may have reset.
  #ifdef TRY_AUTOCONFIG
  #ifdef GPS_10_HZ
  gpsSerial.write(bac_set_10Hz, 14);
  #else
  #ifdef GPS_5_HZ
  gpsSerial.write(bac_set_5Hz, 14);
  #endif
  #endif
  gpsSerial.write(bac_disable_VTG, 16); // Disabling redundant or "debug" GPS messages.
  gpsSerial.write(bac_disable_GGA, 16);
  gpsSerial.write(bac_disable_GLL, 16);
  gpsSerial.write(bac_disable_GSA, 16); // Not truly redundant as it gives altitude info; sacrificing them as I live in a flat land
  gpsSerial.write(bac_disable_GSV, 16);
  gpsSerial.write(bac_baud38400, 28);
  gpsSerial.begin(SW_BAUD_RATE);        // Now, switch back to the higher baud for normal transmission.
  #endif
  new_gps_trasm = false;
  #ifdef LOG_ON_SD
  // SD Card
  if (Serial)
    Serial.print("Initializing SD card...");
  if (!sd.begin(10, SPI_SPEED)) {
    Serial.println("initialization failed!");
    while (1);
  }
  if (Serial)
    Serial.println("initialization done.");
  // Look for the first non-existing "LOG????.txt" file.
  keep_searching_file = true;
  logno = 0;
  while (keep_searching_file)
  {
    sprintf(c_string_number, "%04i", logno);
    for (i_ca = 0; i_ca < 4; i_ca++)
    {
      c_fname[i_ca + 3] = c_string_number[i_ca];
    }
    if (sd.exists(c_fname))
    {
      logno++;
    }
    else
    {
      keep_searching_file = false;
      break;
    }
  }
  myFile = sd.open(c_fname, (O_WRITE | O_CREAT | O_AT_END));
  myFile.println("V-SCATOLA FW 1.0.1 2020");
  myFile.close();
  #ifdef MANUAL_FLUSH_SIZE
  strings_written = 0x00u;
  #endif
  #endif
  ca2read[0] = '\0';
  ca2send[0] = '\0';
  i_ca = 0;
}
 
 
void loop()
{
  #ifdef MANUAL_FLUSH_SIZE                                // manual file opening and closing control. Faster.
  if (strings_written == 0x00u)
  {
    myFile = sd.open(c_fname, (O_WRITE | O_APPEND));
  }
  #endif
  while ((millis() - time_ms) < SAMPLING_DT_MS)           // use SW serial readings to keep IMU pace. Overridden if PRIORITY_GPS.
  {
    serial_bytes_available = gpsSerial.available();
    #ifndef PRIORITY_GPS
      if ((serial_bytes_available > 0))
    #else
      while ((gpsSerial.available()))
    #endif
    {
      c2read = gpsSerial.read();
      if ((c2read == 10) || (i_ca >= (CSTRING_LEN - 1)))  // 10: newline character. New line: store the previous one and send it.
      {
        new_gps_trasm = true;
        memcpy(&ca2send, &ca2read, CSTRING_LEN);
        ca2read[0] = '\0';
        i_ca = 0;
      }
      else
      {
        if (c2read == 36)                                 // 36: "$" char. You might be here because you missed that new line character
        {
          if (!new_gps_trasm)
          {
            new_gps_trasm = true;
            memcpy(&ca2send, &ca2read, CSTRING_LEN);
          }
          i_ca = 0;
        }
        ca2read[i_ca] = c2read;
        ca2read[i_ca + 1] = '\0';
        i_ca++;
      }
    }
  }
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);                                     // First address containing acc data.
  Wire.endTransmission(false);                          // We are NOT releasing the I2C bus.
  Wire.requestFrom(MPU, 14, true);                      // Ask MPU 14 bytes starting from 0x3B. At the end, release I2C bus.
  ax = ((Wire.read()<<8) | (Wire.read())) - OFFSET_AX;  // Read acc + gyro.
  ay = ((Wire.read()<<8) | (Wire.read())) - OFFSET_AY; 
  az = ((Wire.read()<<8) | (Wire.read())) - OFFSET_AZ;
  temp = ((Wire.read()<<8) | (Wire.read()));            // Temperature reading - unused. If removed, read twice before proceeding to GX.
  gx = ((Wire.read()<<8) | (Wire.read())) - OFFSET_GX; 
  gy = ((Wire.read()<<8) | (Wire.read())) - OFFSET_GY; 
  gz = ((Wire.read()<<8) | (Wire.read())) - OFFSET_GZ;
  time_ms += SAMPLING_DT_MS;
  time_true = millis();
  // TODO: tidy up this #ifdef mess...
  if (new_gps_trasm)
  {
    #ifdef LOG_ON_SD
    sd_print_gps();
    #endif
    #ifdef LOG_ON_SERIAL
    serial_print_gps();
      #ifdef MANUAL_FLUSH_SIZE
        strings_written++;
      #endif
    #endif
    new_gps_trasm = false;
  }
  #ifdef LOG_ON_SD
    sd_print_acc();
  #endif
  #ifdef LOG_ON_SERIAL
    serial_print_acc();
    #ifdef MANUAL_FLUSH_SIZE
      strings_written++;
    #endif
  #endif
  #ifdef MANUAL_FLUSH_SIZE
    if (strings_written >= MANUAL_FLUSH_SIZE)
    {
      myFile.close();
      strings_written = 0;
    }
  #endif
}

void sd_print_gps()
{
  #ifndef MANUAL_FLUSH_SIZE
  myFile = sd.open(c_fname, (O_WRITE | O_APPEND));
  #endif
  if (myFile) {
    myFile.print("GPS: T = "); 
    myFile.print(time_true);
    myFile.print("; ");
    myFile.println(ca2send);
    #ifndef MANUAL_FLUSH_SIZE
    myFile.close();
    #endif
  } else {
    if (Serial)
    {
    Serial.println(c_fname);
    Serial.println("error opening txt file.");
    }
  }
}

void sd_print_acc()
{
  #ifndef MANUAL_FLUSH_SIZE
  myFile = sd.open(c_fname, (O_WRITE | O_APPEND));
  #endif
  if (myFile) {
    myFile.print("ACC: T = "); myFile.print(time_true);
    myFile.print("; ax = "); myFile.print(ax);
    myFile.print("; ay = "); myFile.print(ay);
    myFile.print("; az = "); myFile.print(az);
    myFile.print("; gx = "); myFile.print(gx);
    myFile.print("; gy = "); myFile.print(gy);
    myFile.print("; gz = "); myFile.println(gz);
    #ifndef MANUAL_FLUSH_SIZE
    myFile.close();
    #endif
  } else {
    if (Serial)
    {
    Serial.println(c_fname);
    Serial.println("error opening txt file.");
    }
  }
}

void serial_print_gps()
{
  if (Serial)
  {
    Serial.print("GPS: T = "); 
    Serial.print(time_true);
    Serial.print("; ");
    #ifdef DEBUG_SERIAL
    Serial.print(ca2send); Serial.print(";BAV="); Serial.print(serial_bytes_available); Serial.print(";te="); Serial.println(millis());
    #else
    Serial.print(ca2send); Serial.println("");
    #endif
  }
}

void serial_print_acc()
{
  if (Serial)
  {
    Serial.print("ACC: T = "); Serial.print(time_true);
    Serial.print("; ax = "); Serial.print(ax);
    Serial.print("; ay = "); Serial.print(ay);
    Serial.print("; az = "); Serial.print(az);
    Serial.print("; gx = "); Serial.print(gx);
    Serial.print("; gy = "); Serial.print(gy);
    #ifdef DEBUG_SERIAL
    Serial.print("; gz = "); Serial.print(gz); Serial.print(";BAV="); Serial.print(serial_bytes_available); Serial.print(";te="); Serial.println(millis());
    #else
    Serial.print("; gz = "); Serial.println(gz);
    #endif
  }
}
