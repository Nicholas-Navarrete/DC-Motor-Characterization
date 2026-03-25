/*
 * arduino_voltage_logger.ino
 * --------------------------
 * Measures raw ADC on A1 and delta time in microseconds, then transmits
 * as a compact 7-byte binary packet at 115200 baud for maximum throughput.
 *
 * Packet format (7 bytes):
 *   [0]      0xAA           sync byte
 *   [1-2]    uint16_t       raw ADC value (big-endian, 0–1023)
 *   [3-6]    uint32_t       delta time in microseconds (big-endian)
 *
 * Hardware assumptions:
 *   - Arduino Uno / Nano (ATmega328P, 16 MHz, 10-bit ADC, 5V)
 *   - Change VREF in the Python script if using a 3.3V board
 *
 * Timing notes:
 *   - ADC prescaler set to 16 → ~13 µs per conversion @ 16 MHz
 *   - Binary packet is 7 bytes vs ~22 bytes ASCII → ~3x less serial time
 *   - No float math, no String formatting on the Arduino side
 */

static const uint8_t ANALOG_PIN = A1;
static const uint8_t SYNC_BYTE  = 0xAA;

static unsigned long lastMicros = 0;

void setup() {
  Serial.begin(500000);

  // ADC prescaler = 16 → faster conversions (~13 µs vs ~112 µs default)
  ADCSRA = (ADCSRA & ~0x07) | 0x04;

  // Warm-up read
  analogRead(ANALOG_PIN);

  lastMicros = micros();
}

void loop() {
  // 1. Timestamp before conversion
  unsigned long nowMicros = micros();

  // 2. Read ADC (10-bit, 0–1023)
  uint16_t raw = (uint16_t)analogRead(ANALOG_PIN);

  // 3. Delta time in microseconds
  uint32_t deltaMicros = (uint32_t)(nowMicros - lastMicros);
  lastMicros = nowMicros;

  // 4. Send 7-byte binary packet (all big-endian for easy Python struct unpack)
  Serial.write(SYNC_BYTE);
  Serial.write((uint8_t)(raw >> 8));          // ADC high byte
  Serial.write((uint8_t)(raw & 0xFF));        // ADC low byte
  Serial.write((uint8_t)(deltaMicros >> 24)); // delta µs byte 3
  Serial.write((uint8_t)(deltaMicros >> 16)); // delta µs byte 2
  Serial.write((uint8_t)(deltaMicros >> 8));  // delta µs byte 1
  Serial.write((uint8_t)(deltaMicros));       // delta µs byte 0
}
