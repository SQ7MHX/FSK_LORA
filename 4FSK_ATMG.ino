//APRS  https://github.com/mikaelnousiainen/RS41ng 
#include <TinyGPSPlus.h>
https://github.com/etherkit/Si5351Arduino
#include <si5351.h>
#include <SoftwareSerial.h>
#include "Wire.h"
//https://github.com/projecthorus/horusbinary
#include "horus_l2.h"
#include <SPI.h>
//https://github.com/sandeepmistry/arduino-LoRa
#include <LoRa.h>

///////////////////////////////
#define LORA
//#define FSK
///////////////////////////////
#if defined (FSK)
Si5351 si5351;
#endif
SoftwareSerial GPSSerial(2, 3);  //GPS Serial port, RX on pin 2, TX on pin 3

static uint8_t GPS_airborne[44] = { 0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0x01, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x56, 0xD6 };

TinyGPSPlus gps;

// APRS

#define APRS_CALLSIGN "N0CALL"
#define APRS_SSID "12"
// See APRS symbol table documentation in: http://www.aprs.org/symbols/symbolsX.txt
#define APRS_SYMBOL_TABLE '/'  // '/' denotes primary and '\\' denotes alternate APRS symbol table
#define APRS_SYMBOL 'O'
#define APRS_COMMENT ""
#define APRS_RELAYS "WIDE1-1"
#define APRS_DESTINATION "APZ41N"
#define APRS_DESTINATION_SSID '0'
#define RADIO_PAYLOAD_MAX_LENGTH 256
#define RADIO_PAYLOAD_HEADER 25

// APRS FUNCTIONS //
#if defined (LORA)
void aprs_generate_timestamp();
size_t aprs_generate_position(uint8_t *payload, size_t length, char symbol_table, char symbol, bool include_timestamp, char *comment);
void convert_degrees_to_dmh(long x, uint8_t *degrees, uint8_t *minutes, uint8_t *h_minutes);
void generate_header();
void generate_aprsmessage();

char timestamp[15];
uint8_t la_degrees, lo_degrees;
uint8_t la_minutes, la_h_minutes, lo_minutes, lo_h_minutes;
uint8_t aprs_header[RADIO_PAYLOAD_HEADER];
uint8_t aprs_packet[RADIO_PAYLOAD_MAX_LENGTH];
uint8_t aprs_message[RADIO_PAYLOAD_HEADER + RADIO_PAYLOAD_MAX_LENGTH];
int aprsmsglenght;
int frame_counter = 0;
int temp_c = 0;
int batvoltage = 0;
// RADIO
void LoraAPRSsend(const long frequency, const int SF, const int CR);

int Loramode = 1;

// LORA1 //

const long frequencyL1 = 433.775E6;  // LoRa1 Frequency
const int SFL1 = 12;
const int CRL1 = 5;

// LORA2
const long frequencyL2 = 434.855E6;  // LoRa2 Frequency
const int SFL2 = 9;
const int CRL2 = 7;

// LORA pins
const int csPin = 4;     // LoRa radio chip select
const int resetPin = 5;  // LoRa radio reset
const int irqPin = 6;    // change for your board; must be a hardware interrupt pin
// const int DIO1 = 7;
#endif
// GPS//ZMIENNE/////////////////////////////////////////////////////////////
int32_t rawlat;
int32_t rawlon;
double latf;
double lonf;
int8_t usedsat;
uint8_t hour_int;
uint8_t min_int;
uint8_t sek_int;
uint32_t altm;
uint32_t altft;
uint8_t SPD;
// ZMIENNE TIMERY/////////////////////////////////////////////////////////////
int new_frame = 0;
int previous_min = 0;
#if defined (FSK)
// Horus v2 Mode 1 (32-byte) Binary Packet
struct HorusBinaryPacketV2 {
  uint16_t PayloadID;
  uint16_t Counter;
  uint8_t Hours;
  uint8_t Minutes;
  uint8_t Seconds;
  float Latitude;
  float Longitude;
  uint16_t Altitude;
  uint8_t Speed;  // Speed in Knots (1-255 knots)
  uint8_t Sats;
  int8_t Temp;          // Twos Complement Temp value.
  uint8_t BattVoltage;  // 0 = 0v, 255 = 5.0V, linear steps in-between.
  // The following 9 bytes (up to the CRC) are user-customizable. The following just
  // provides an example of how they could be used.
  uint8_t dummy1;     // unsigned int
  float dummy2;       // Float
  uint8_t dummy3;     // battery voltage test
  uint8_t dummy4;     // divide by 10
  uint16_t dummy5;    // divide by 100
  uint16_t Checksum;  // CRC16-CCITT Checksum.
} __attribute__((packed));

// Buffers and counters.
char rawbuffer[128];        // Buffer to temporarily store a raw binary packet.
char codedbuffer[128];      // Buffer to store an encoded binary packet
char debugbuffer[256];      // Buffer to store debug strings
uint16_t packet_count = 1;  // Packet counter
int coded_len;
// 4FSK FUNCTION
int build_horus_binary_packet_v2(char *buffer);
unsigned int crc16(unsigned char *string, unsigned int len);
uint16_t _crc_xmodem_update(uint16_t crc, uint8_t data);
void fsk4_tone(uint8_t i);
void fsk4_idle();
void fsk4_preamble(uint8_t len);
size_t fsk4_writebyte(uint8_t b);
int fsk4_write(char *buff, size_t len);
void send_4fsk();
#endif
//GPS DISPLAY
void displayInfo();

void setup() {
  //LED
  pinMode(A2, OUTPUT);
  digitalWrite(A2, HIGH);
  //SERIAL

  Serial.begin(9600);  //USB Serial port
  Serial.setTimeout(2000);
  GPSSerial.begin(9600);

  delay(2000);
  for (uint8_t ih = 0; ih < 2; ih++) {
    for (uint8_t ig = 0; ig < 44; ig++) {

      GPSSerial.write(GPS_airborne[ig]);
    }
    if (ih == 0) {
      delay(900);
    }
    delay(100);
  }

#if defined (LORA)
  LoRa.setPins(csPin, resetPin, irqPin);
  generate_header();
#endif
  digitalWrite(A2, LOW);
}

void loop() {
  //GPS RX
  while (GPSSerial.available() > 0)
    if (gps.encode(GPSSerial.read()))
      new_frame = 1;

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS detected: check wiring."));
  }
  if (gps.satellites.value() <= 3 && new_frame == 1) {
    new_frame = 0;
    digitalWrite(A2, LOW);
  }
  if (gps.location.isValid() == true && (gps.time.isValid()) == true && gps.altitude.isValid() == true && gps.location.age() < 1000 && new_frame == 1 && gps.satellites.value() > 3) {
    rawlat = gps.location.lat() * 1000000;
    rawlon = gps.location.lng() * 1000000;
    latf = gps.location.lat();
    lonf = gps.location.lng();
    hour_int = (gps.time.hour());
    min_int = (gps.time.minute());
    sek_int = (gps.time.second());
    altm = (gps.altitude.meters());
    altft = (gps.altitude.feet());
    usedsat = (gps.satellites.value());
    SPD = (gps.speed.kmph());
#if defined (FSK)
    // 4FSK/////////////////////////////////////////////////////
    int pkt_len = build_horus_binary_packet_v2(rawbuffer);
    // Apply Encoding
    coded_len = horus_l2_encode_tx_packet((unsigned char *)codedbuffer, (unsigned char *)rawbuffer, pkt_len);
#endif
#if defined (LORA)
    aprs_generate_timestamp();
    convert_degrees_to_dmh(rawlat, &la_degrees, &la_minutes, &la_h_minutes);
    convert_degrees_to_dmh(rawlon, &lo_degrees, &lo_minutes, &lo_h_minutes);
    aprs_generate_position(aprs_packet, sizeof(aprs_packet), APRS_SYMBOL_TABLE, APRS_SYMBOL, true, APRS_COMMENT);
    generate_aprsmessage();
    //Serial.printf("%s", aprs_message);
    //Serial.printf("\r\n");
#endif
    if (altm < 500) {
      digitalWrite(A2, HIGH);
    }
  }
  if (gps.satellites.value() > 3 && gps.location.age() < 2000) {
    if (min_int == previous_min + 1 && sek_int % 5 == 0) {
      // TX MODULE INIT ////////////////////////////////
      digitalWrite(A2, LOW);
      previous_min = min_int;
      delay(100);
#if defined (FSK)
      // 4FSK
      si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
      si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_6MA);  // Set for max power if desired
      si5351.set_freq(14407500000ULL, SI5351_CLK0);
      si5351.output_enable(SI5351_CLK0, 1);  // Ensable the clock initially
      Serial.println("4FSK");
      send_4fsk();
      // DELAY AFTER TX //
      delay(100);
      // RADIO OFF //
      si5351.output_enable(SI5351_CLK0, 0);  // Disable the clock initially
#endif
//LORA
#if defined (LORA)
      if (Loramode == 1) {
        LoraAPRSsend(frequencyL1, SFL1, CRL1);
        frame_counter = frame_counter + 1;
      }

      if (Loramode == 2) {
        LoraAPRSsend(frequencyL2, SFL2, CRL2);
        frame_counter = frame_counter + 1;
      }
#endif
      delay(100);
      // END OF TX
    }
  }
}
// 4FSK FUNCTIONS
#if defined (FSK)
int build_horus_binary_packet_v2(char *buffer) {
  // Generate a Horus Binary v2 packet, and populate it with data.
  // The assignments in this function should be replaced with real data

  struct HorusBinaryPacketV2 BinaryPacketV2;

  BinaryPacketV2.PayloadID = 0;  // 0 = 4FSKTEST-V2. Refer https://github.com/projecthorus/horusdemodlib/blob/master/payload_id_list.txt
  BinaryPacketV2.Counter = packet_count;
  BinaryPacketV2.Hours = hour_int;
  BinaryPacketV2.Minutes = min_int;
  BinaryPacketV2.Seconds = sek_int;
  BinaryPacketV2.Latitude = latf;
  BinaryPacketV2.Longitude = lonf;
  BinaryPacketV2.Altitude = altm;
  BinaryPacketV2.Speed = SPD;
  BinaryPacketV2.BattVoltage = 0;
  BinaryPacketV2.Sats = usedsat;
  BinaryPacketV2.Temp = 0;
  // Custom section. This is an example only, and the 9 bytes in this section can be used in other
  // ways. Refer here for details: https://github.com/projecthorus/horusdemodlib/wiki/5-Customising-a-Horus-Binary-v2-Packet
  BinaryPacketV2.dummy1 = 0;    // uint8
  BinaryPacketV2.dummy2 = 0.0;  // float32
  BinaryPacketV2.dummy3 = 0;    // uint8 - interpreted as a battery voltage 0-5V
  BinaryPacketV2.dummy4 = 0;    // uint8 - interpreted as a fixed-point value (div/10)
  BinaryPacketV2.dummy5 = 0;    // uint16 - interpreted as a fixed-point value (div/100)

  BinaryPacketV2.Checksum = (uint16_t)crc16((unsigned char *)&BinaryPacketV2, sizeof(BinaryPacketV2) - 2);

  memcpy(buffer, &BinaryPacketV2, sizeof(BinaryPacketV2));

  return sizeof(struct HorusBinaryPacketV2);
}


uint16_t _crc_xmodem_update(uint16_t crc, uint8_t data) {
  int i;
  crc = crc ^ ((uint16_t)data << 8);
  for (i = 0; i < 8; i++) {
    if (crc & 0x8000)
      crc = (crc << 1) ^ 0x1021;
    else
      crc <<= 1;
  }
  return crc;
}

unsigned int crc16(unsigned char *string, unsigned int len) {
  unsigned int i;
  unsigned int crc;
  crc = 0xFFFF;  // Standard CCITT seed for CRC16.
  // Calculate the sum, ignore $ sign's
  for (i = 0; i < len; i++) {
    crc = _crc_xmodem_update(crc, (uint8_t)string[i]);
  }
  return crc;
}

void fsk4_tone(uint8_t i) {
  uint32_t start = micros();
  if (i == 0) {
    si5351.set_freq(14407500000ULL, SI5351_CLK0);
  }
  if (i == 1) {
    si5351.set_freq(14407510000ULL, SI5351_CLK0);
  }
  if (i == 2) {
    si5351.set_freq(14407520000ULL, SI5351_CLK0);
  }
  if (i == 3) {
    si5351.set_freq(14407530000ULL, SI5351_CLK0);
  }

  while (micros() - start < 20000) {
  }
}

void fsk4_idle() {
  si5351.output_enable(SI5351_CLK0, 1);  // Enable the clock initially
  fsk4_tone(0);
}

void fsk4_preamble(uint8_t len) {
  int k;
  for (k = 0; k < len; k++) {
    fsk4_writebyte(0x1B);
  }
}

size_t fsk4_writebyte(uint8_t b) {
  int k;
  // Send symbols MSB first.
  for (k = 0; k < 4; k++) {
    // Extract 4FSK symbol (2 bits)
    uint8_t symbol = (b & 0xC0) >> 6;
    // Modulate
    fsk4_tone(symbol);
    // Shift to next symbol.
    b = b << 2;
  }

  return (1);
}

int fsk4_write(char *buff, size_t len) {

  size_t n = 0;
  for (size_t i = 0; i < len; i++) {
    n += fsk4_writebyte(buff[i]);
  }
  si5351.output_enable(SI5351_CLK0, 0);
  return n;
}

void send_4fsk() {

// Reset the tone to the base frequency and turn on the output
// Horus Binary V2
#if defined(DEVMODE)
  Serial.println(F("Generating Horus Binary v2 Packet"));
// Generate packet
#endif
  int pkt_len = build_horus_binary_packet_v2(rawbuffer);
  // Debugging
#if defined(DEVMODE)
  Serial.print(F("Uncoded Length (bytes): "));
  Serial.println(pkt_len);
  Serial.print("Uncoded: ");
  // PrintHex(rawbuffer, pkt_len, debugbuffer);
  Serial.println(debugbuffer);
#endif
  // Apply Encoding
  int coded_len = horus_l2_encode_tx_packet((unsigned char *)codedbuffer, (unsigned char *)rawbuffer, pkt_len);
  // Debugging
#if defined(DEVMODE)
  Serial.print(F("Encoded Length (bytes): "));
  Serial.println(coded_len);
  Serial.print("Coded: ");
  // PrintHex(codedbuffer, coded_len, debugbuffer);
  Serial.println(debugbuffer);
  // Transmit!
  Serial.println(F("Transmitting Horus Binary v2 Packet"));
#endif
  // send out idle condition for 1000 ms
  fsk4_idle();
  delay(2000);
  fsk4_preamble(8);
  fsk4_write(codedbuffer, coded_len);

  delay(1000);

  packet_count++;
  }
#endif

// LORA
#if defined (LORA)

void LoraAPRSsend(const long frequency, const int SF, const int CR) {
  LoRa.begin(frequency);
  LoRa.setSpreadingFactor(SF);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(CR);
  LoRa.enableCrc();
  LoRa.idle();             // set standby mode
  LoRa.disableInvertIQ();  // normal mode
  // String data = message.encode();
  //  message += "I'm a Node! ";
  //  message += millis();
  LoRa.beginPacket();
  LoRa.write('<');
  LoRa.write(0xFF);
  LoRa.write(0x01);
  LoRa.write((const uint8_t *)aprs_message, aprsmsglenght);
  LoRa.endPacket();
  LoRa.idle();  // set standby mode
  LoRa.end();
  // send a message
  // LoRa_sendMessage(data); // send a message
  // Serial.println("Send Message!");
}

// APRS Parser//
void convert_degrees_to_dmh(long x, uint8_t *degrees, uint8_t *minutes, uint8_t *h_minutes) {
  uint8_t sign = (uint8_t)(x > 0 ? 1 : 0);
  if (!sign) {
    x = -(x);
  }
  *degrees = (int8_t)(x / 1000000);
  x = x - (*degrees * 1000000);
  x = (x)*60 / 10000;
  *minutes = (uint8_t)(x / 100);
  *h_minutes = (uint8_t)(x - (*minutes * 100));
  if (!sign) {
    *degrees = -*degrees;
  }
}

void aprs_generate_timestamp() {
  snprintf(timestamp, sizeof(timestamp), "/%02d%02d%02dh", hour_int, min_int, sek_int);
}

size_t aprs_generate_position(uint8_t *payload, size_t length,
                              char symbol_table, char symbol, bool include_timestamp, char *comment) {

  uint8_t la_degrees, lo_degrees;
  uint8_t la_minutes, la_h_minutes, lo_minutes, lo_h_minutes;

  convert_degrees_to_dmh(rawlat, &la_degrees, &la_minutes, &la_h_minutes);
  convert_degrees_to_dmh(rawlon, &lo_degrees, &lo_minutes, &lo_h_minutes);

  if (include_timestamp) {
    aprs_generate_timestamp();
  } else {
    strncpy(timestamp, "!", sizeof(timestamp));
  }

  return snprintf((char *)payload,
                  length,
                  ("%s%02d%02d.%02u%c%c%03d%02u.%02u%c%c/A=%06d P%dS%dT%dV%d %s"),
                  timestamp,
                  abs(la_degrees), la_minutes, la_h_minutes,
                  la_degrees > 0 ? 'N' : 'S',
                  symbol_table,
                  abs(lo_degrees), lo_minutes, lo_h_minutes,
                  lo_degrees > 0 ? 'E' : 'W',
                  symbol,
                  altft,
                  frame_counter,
                  usedsat,
                  temp_c,
                  batvoltage,
                  comment);
}
void generate_aprsmessage() {

  aprsmsglenght = snprintf((char *)aprs_message, sizeof(aprs_message), ("%s:%s"), aprs_header, aprs_packet);
}

void generate_header() {

  snprintf((char *)aprs_header, sizeof(aprs_header), ("%s-%s>%s,%s"), APRS_CALLSIGN, APRS_SSID, APRS_DESTINATION, APRS_RELAYS);
}
//END LORA
#endif

void displayInfo() {
  Serial.print(F("Location: "));
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid()) {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}
