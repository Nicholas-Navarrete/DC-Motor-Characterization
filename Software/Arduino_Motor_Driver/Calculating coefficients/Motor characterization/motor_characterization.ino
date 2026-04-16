/*
 * motor_characterization.ino
 * --------------------------
 * Drives a brushed DC motor with a 20 kHz square-wave PWM (100% for 5 s,
 * then 0% for 5 s, repeating) via an H-bridge on pin D8.
 *
 * Simultaneously samples:
 *   A0 – shunt resistor (current sense)
 *   A1 – optical encoder (tachometer)
 *
 * Every measurement group is transmitted as a compact 11-byte binary packet
 * at 500 000 baud for maximum throughput.
 *
 * Packet format (11 bytes, all big-endian):
 *   [0]      0xBB           sync byte
 *   [1-2]    uint16_t       A0 raw ADC  (shunt / current)
 *   [3-4]    uint16_t       A1 raw ADC  (encoder)
 *   [5]      uint8_t        commanded duty cycle 0-100 (%)
 *   [6-9]    uint32_t       delta time in microseconds
 *   [10]     0xEE           end byte  (sanity check in Python)
 *
 * Hardware:
 *   Arduino Uno (ATmega328P, 16 MHz, 10-bit ADC, 5 V)
 *   Timer 1 is repurposed for 20 kHz PWM on OC1B → Arduino pin D10.
 *   D8 is used instead, so we use Timer 1 in Fast PWM mode with OC1A on D9
 *   and OC1B on D10.  Per the requirements pin D8 is specified; however
 *   the ATmega328P's hardware 20 kHz PWM is most cleanly generated on D9
 *   (OC1A) or D10 (OC1B).  We therefore use D9 as the PWM output and leave
 *   this note for the integrator to adjust the H-bridge wiring accordingly.
 *   If the H-bridge enable is hard-wired to D8, jumper D9 → D8.
 *
 * ADC prescaler is set to 16 (≈ 13 µs / conversion) for speed.
 * Two ADC reads per loop ≈ 26 µs + serial write ≈ a few µs at 500 kbaud.
 */

// ── Pin assignments ───────────────────────────────────────────────────────────
static const uint8_t PIN_CURRENT  = A0;   // shunt voltage
static const uint8_t PIN_ENCODER  = A1;   // optical encoder
static const uint8_t PIN_PWM      = 9;    // OC1A – 20 kHz capable

// ── Packet framing ────────────────────────────────────────────────────────────
static const uint8_t SYNC_BYTE = 0xBB;
static const uint8_t END_BYTE  = 0xEE;

// ── Square-wave timing ────────────────────────────────────────────────────────
static const unsigned long ON_TIME_MS  = 5000UL;   // 5 s full power
static const unsigned long OFF_TIME_MS = 5000UL;   // 5 s coast

// ── State ─────────────────────────────────────────────────────────────────────
static unsigned long lastMicros      = 0;
static unsigned long phaseStartMs    = 0;
static uint8_t       dutyPercent     = 100;   // current commanded duty (0 or 100)

// ─────────────────────────────────────────────────────────────────────────────
// Timer 1 setup: Fast PWM, non-inverting on OC1A (D9)
//   f_PWM = f_clk / (prescaler × (TOP+1))
//   Target: 20 000 Hz  →  TOP = 16 000 000 / (1 × 20 000) − 1 = 799
// ─────────────────────────────────────────────────────────────────────────────
static const uint16_t PWM_TOP = 799;   // gives exactly 20 kHz

void setupTimer1_20kHz() {
  // Stop timer, clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  ICR1   = PWM_TOP;

  // Fast PWM, TOP = ICR1; OC1A non-inverting (COM1A1=1)
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  // WGM13:WGM12 set for Fast PWM / ICR1; prescaler = 1
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);

  OCR1A = PWM_TOP;   // start at 100% duty
}

// Set duty cycle 0-100 %
inline void setDuty(uint8_t pct) {
  OCR1A = ((uint32_t)pct * PWM_TOP) / 100UL;
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(500000);

  // ADC prescaler = 16 → ~13 µs per conversion
  ADCSRA = (ADCSRA & ~0x07) | 0x04;

  // Warm-up reads
  analogRead(PIN_CURRENT);
  analogRead(PIN_ENCODER);

  pinMode(PIN_PWM, OUTPUT);
  setupTimer1_20kHz();

  dutyPercent  = 100;
  phaseStartMs = millis();
  lastMicros   = micros();
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  // ── 1. Update square-wave phase ───────────────────────────────────────────
  unsigned long nowMs   = millis();
  unsigned long elapsed = nowMs - phaseStartMs;

  if (dutyPercent == 100 && elapsed >= ON_TIME_MS) {
    dutyPercent  = 0;
    setDuty(0);
    phaseStartMs = nowMs;
  } else if (dutyPercent == 0 && elapsed >= OFF_TIME_MS) {
    dutyPercent  = 100;
    setDuty(100);
    phaseStartMs = nowMs;
  }

  // ── 2. Timestamp + ADC reads ──────────────────────────────────────────────
  unsigned long nowUs    = micros();
  uint16_t adcCurrent    = (uint16_t)analogRead(PIN_CURRENT);
  uint16_t adcEncoder    = (uint16_t)analogRead(PIN_ENCODER);
  uint32_t deltaMicros   = (uint32_t)(nowUs - lastMicros);
  lastMicros             = nowUs;

  // ── 3. Transmit 11-byte packet ────────────────────────────────────────────
  Serial.write(SYNC_BYTE);

  Serial.write((uint8_t)(adcCurrent >> 8));        // A0 hi
  Serial.write((uint8_t)(adcCurrent & 0xFF));      // A0 lo

  Serial.write((uint8_t)(adcEncoder >> 8));        // A1 hi
  Serial.write((uint8_t)(adcEncoder & 0xFF));      // A1 lo

  Serial.write(dutyPercent);                       // commanded duty %

  Serial.write((uint8_t)(deltaMicros >> 24));      // Δt byte 3
  Serial.write((uint8_t)(deltaMicros >> 16));      // Δt byte 2
  Serial.write((uint8_t)(deltaMicros >> 8));       // Δt byte 1
  Serial.write((uint8_t)(deltaMicros));            // Δt byte 0

  Serial.write(END_BYTE);
}
