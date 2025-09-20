#include <Arduino.h>
#include <SPI.h>

// EEPROM kommandoer
#define EEPROM_WREN   0x06
#define EEPROM_RDSR   0x05
#define EEPROM_READ   0x03
#define EEPROM_WRITE  0x02

// Chip Select pin (PA4 = pin 4 i Arduino core mapping for Tiny3224)
const int EEPROM_CS = PIN_PA4;

// Prototyper
void EEPROM_writeEnable();
uint8_t EEPROM_readStatus();
void EEPROM_writeByte(uint16_t addr, uint8_t data);
uint8_t EEPROM_readByte(uint16_t addr);

void setup() {
  pinMode(EEPROM_CS, OUTPUT);
  digitalWrite(EEPROM_CS, HIGH);

  Serial.begin(460800);
  while (!Serial);

  SPI.begin();   // MOSI=PA1, MISO=PA2, SCK=PA3

  delay(10);

  // Test: skriv og læs tilbage
  uint16_t addr = 0x0010;
  uint8_t value = 0xCE;

  EEPROM_writeByte(addr, value);
  uint8_t readback = EEPROM_readByte(addr);

  Serial.print(F("Wrote 0x"));
  Serial.print(value, HEX);
  Serial.print(F(", Read back 0x"));
  Serial.println(readback, HEX);
}

void loop() {
  delay(1000);
  uint8_t val = EEPROM_readByte(0x0010);
  Serial.print(F("Addr 0x0010 = 0x"));
  Serial.println(val, HEX);
}

// ==========================================
// EEPROM funktioner

void EEPROM_writeEnable() {
  digitalWrite(EEPROM_CS, LOW);
  SPI.transfer(EEPROM_WREN);
  digitalWrite(EEPROM_CS, HIGH);
}

uint8_t EEPROM_readStatus() {
  uint8_t status;
  digitalWrite(EEPROM_CS, LOW);
  SPI.transfer(EEPROM_RDSR);
  status = SPI.transfer(0xFF);
  digitalWrite(EEPROM_CS, HIGH);
  return status;
}

void EEPROM_writeByte(uint16_t addr, uint8_t data) {
  EEPROM_writeEnable();

  digitalWrite(EEPROM_CS, LOW);
  SPI.transfer(EEPROM_WRITE);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
  SPI.transfer(data);
  digitalWrite(EEPROM_CS, HIGH);

  // vent på at skrivning er færdig
  while (EEPROM_readStatus() & 0x01);
}

uint8_t EEPROM_readByte(uint16_t addr) {
  uint8_t data;
  digitalWrite(EEPROM_CS, LOW);
  SPI.transfer(EEPROM_READ);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
  data = SPI.transfer(0xFF);
  digitalWrite(EEPROM_CS, HIGH);
  return data;
}
