#define F_CPU 8000000UL

#include <Arduino.h>
#include <TinyI2CMaster.h>
#include <EEPROM.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>

#define EEPROM_ADDR_MODE 0

uint8_t currentMode = 0;

#define OLED_ADDR   0x3C
#define OLED_WIDTH  128
#define OLED_HEIGHT 64
#define OLED_PAGES  8
#define SAMPLES     32
#define ADC_PIN     A3
#define BUZZ_PIN    1

uint8_t smp[SAMPLES];

const uint8_t Init[] PROGMEM = {
  0xAE,
  0xD5, 0x80,
  0xA8, 0x3F,
  0xD3, 0x00,
  0x40,
  0x8D, 0x14,
  0xA1,
  0xC8,
  0xDA, 0x12,
  0x81, 0xCF,
  0xD9, 0xF1,
  0xDB, 0x40,
  0xA4,
  0xA6,
  0xAF
};

void oledCommand(uint8_t c) {
  TinyI2C.start(OLED_ADDR, 0);
  TinyI2C.write(0x00);
  TinyI2C.write(c);
  TinyI2C.stop();
}

void InitDisplay() {
  TinyI2C.init();
  TinyI2C.start(OLED_ADDR, 0);
  TinyI2C.write(0x00);
  for (uint8_t i = 0; i < sizeof(Init); i++) {
    TinyI2C.write(pgm_read_byte(&Init[i]));
  }
  TinyI2C.stop();
}

void clearDisplay() {
  for (uint8_t page = 0; page < OLED_PAGES; page++) {
    oledCommand(0xB0 + page);
    oledCommand(0x00);
    oledCommand(0x10);
    TinyI2C.start(OLED_ADDR, 0);
    TinyI2C.write(0x40);
    for (uint8_t col = 0; col < OLED_WIDTH; col++) TinyI2C.write(0x00);
    TinyI2C.stop();
  }
}

void drawWaveform(uint8_t min_val, uint8_t max_val) {
  int prev_x = -1, prev_y = -1;

  for (uint8_t page = 0; page < OLED_PAGES; page++) {
    uint8_t page_mask[OLED_WIDTH];
    memset(page_mask, 0, OLED_WIDTH);

    for (uint8_t i = 0; i < SAMPLES; i++) {
      int x = map(i, 0, SAMPLES - 1, 0, OLED_WIDTH - 1);
      int y_mapped = map(smp[i], min_val, max_val, 0, OLED_HEIGHT - 1);
      int y = (OLED_HEIGHT - 1) - y_mapped;

      if (i > 0 && prev_x >= 0) {
        int x0 = prev_x, y0 = prev_y;
        int x1 = x, y1 = y;
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        while (true) {
          if (x0 >= 0 && x0 < OLED_WIDTH && y0 >= 0 && y0 < OLED_HEIGHT) {
            uint8_t p = y0 / 8;
            if (p == page) page_mask[x0] |= (1 << (y0 % 8));
          }
          if (x0 == x1 && y0 == y1) break;
          int e2 = 2 * err;
          if (e2 > -dy) { err -= dy; x0 += sx; }
          if (e2 < dx)  { err += dx; y0 += sy; }
        }
      }

      prev_x = x;
      prev_y = y;
    }

    oledCommand(0xB0 + page);
    oledCommand(0x00);
    oledCommand(0x10);

    TinyI2C.start(OLED_ADDR, 0);
    TinyI2C.write(0x40);
    for (uint8_t col = 0; col < OLED_WIDTH; col++) {
      TinyI2C.write(page_mask[col]);
    }
    TinyI2C.stop();
  }
}

volatile uint8_t bursts_remaining = 0;
volatile unsigned long next_burst_time = 0;
volatile unsigned long burst_spacing_us = 0;
volatile unsigned long burst_beep_len = 0;
volatile uint16_t burst_tone_freq = 2600;

volatile bool tone_running = false;
volatile unsigned long tone_end_time = 0;

inline void buzzer_pin_off() {
  PORTB &= ~_BV(PORTB1);
}

void startTone(uint16_t freq_hz) {
  if (freq_hz == 0) return;

  const uint16_t ps_vals[] = {1, 2, 4, 8, 16, 32, 64};
  uint16_t chosen_ps = 0, ocr_val = 0;

  for (uint8_t i = 0; i < (sizeof(ps_vals) / sizeof(ps_vals[0])); ++i) {
    uint32_t tmp = (uint32_t)F_CPU / (uint32_t)ps_vals[i] / (uint32_t)freq_hz;
    if (tmp >= 2 && tmp <= 256) {
      chosen_ps = ps_vals[i];
      ocr_val = (uint16_t)(tmp - 1);
      break;
    }
  }
  if (chosen_ps == 0) {
    chosen_ps = 64;
    ocr_val = 255;
  }

  uint8_t duty = ((uint16_t)ocr_val + 1) / 2 - 1;
  if (duty < 1) duty = 1;

  pinMode(BUZZ_PIN, OUTPUT);

  TIFR = _BV(TOV1) | _BV(OCF1A);

  OCR1C = (uint8_t)ocr_val;
  OCR1A = duty;

  uint8_t cs_bits = 0;
  switch (chosen_ps) {
    case 1:  cs_bits = _BV(CS10); break;
    case 2:  cs_bits = _BV(CS11); break;
    case 4:  cs_bits = _BV(CS11) | _BV(CS10); break;
    case 8:  cs_bits = _BV(CS12); break;
    case 16: cs_bits = _BV(CS12) | _BV(CS10); break;
    case 32: cs_bits = _BV(CS12) | _BV(CS11); break;
    case 64: cs_bits = _BV(CS12) | _BV(CS11) | _BV(CS10); break;
    default: cs_bits = _BV(CS10); break;
  }

  TCCR1 = _BV(PWM1A) | _BV(COM1A1) | cs_bits;
  tone_running = true;
}

void stopTone() {
  TCCR1 = 0;
  GTCCR = 0;
  tone_running = false;
  buzzer_pin_off();
}

void scheduleBuzzerBursts(uint8_t ticks, unsigned long spacing_us, unsigned long beep_len_us, uint16_t tone_freq_hz) {
  if (ticks == 0) return;
  noInterrupts();
  bursts_remaining = ticks;
  burst_spacing_us = spacing_us;
  burst_beep_len = beep_len_us;
  burst_tone_freq = tone_freq_hz;
  next_burst_time = micros();
  interrupts();
}

void buzzerSchedulerRun() {
  unsigned long now = micros();

  if (tone_running) {
    if ((long)(now - tone_end_time) >= 0) {
      stopTone();
      next_burst_time = tone_end_time + burst_spacing_us;
    } else return;
  }

  if (bursts_remaining > 0 && (long)(now - next_burst_time) >= 0) {
    startTone(burst_tone_freq);
    tone_end_time = now + burst_beep_len;
    bursts_remaining--;
  }
}

volatile unsigned long buzz_until = 0;

void setup_waveform() {
  InitDisplay();
  clearDisplay();

  ADCSRA &= ~(_BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0));
  ADCSRA |= _BV(ADPS1);

  pinMode(BUZZ_PIN, OUTPUT);
  buzzer_pin_off();
}

void loop_waveform() {
  unsigned long t0 = micros();
  for (uint16_t i = 0; i < SAMPLES; i++) {
    uint32_t temp = 0;
    for (uint8_t j = 0; j < 2; j++) temp += analogRead(ADC_PIN);
    smp[i] = (uint8_t)map(temp / 2, 0, 1023, 0, 240);
  }
  unsigned long dt = micros() - t0;

  uint8_t min_val = 255, max_val = 0;
  for (uint16_t i = 0; i < SAMPLES; i++) {
    if (smp[i] < min_val) min_val = smp[i];
    if (smp[i] > max_val) max_val = smp[i];
  }
  if (min_val == max_val) {
    if (min_val > 0) min_val--;
    if (max_val < 255) max_val++;
  }

  static uint8_t display_counter = 0;
  if (display_counter++ % 5 == 0) {
    drawWaveform(min_val, max_val);
  }

  unsigned long sum = 0;
  for (uint16_t i = 0; i < SAMPLES; i++) sum += smp[i];
  int mean = (int)(sum / SAMPLES);

  const int thresh = 0;
  int prev_sign = 0, crossings = 0;
  for (uint16_t i = 0; i < SAMPLES; i++) {
    int diff = (int)smp[i] - mean;
    int sign = (diff > thresh) ? 1 : (diff < -thresh ? -1 : 0);
    if (prev_sign != 0 && sign != 0 && sign != prev_sign) crossings++;
    if (sign != 0) prev_sign = sign;
  }

  unsigned long freq_hz = 0;
  if (dt > 0 && crossings > 0) {
    freq_hz = (unsigned long)crossings * 1000000UL / (2UL * dt);
  }

  bool has_short_change = false;
  const uint8_t small_diff_threshold = 3;
  for (uint16_t i = 1; i < SAMPLES; i++) {
    if (abs((int)smp[i] - (int)smp[i - 1]) >= small_diff_threshold) {
      has_short_change = true;
    }
  }

  unsigned long now = micros();
  if (freq_hz > 0 && freq_hz <= 8000) {
    unsigned long spacing_us = 1000000UL / freq_hz;
    if (spacing_us > 100000UL) spacing_us = 100000UL;
    uint8_t ticks = max((uint8_t)1, min((uint8_t)3, (uint8_t)(100000UL / spacing_us)));
    unsigned long beep_len = constrain(spacing_us / 2, 5000UL, 50000UL);
    scheduleBuzzerBursts(ticks, spacing_us, beep_len, 2600);
    buzz_until = now + 500000UL;
  } else if (max_val - min_val >= 3 || has_short_change) {
    scheduleBuzzerBursts(5, 10000UL, 5000UL, 2600);
    buzz_until = now + 500000UL;
  } else if (now < buzz_until) {
    if (bursts_remaining == 0) {
      scheduleBuzzerBursts(5, 10000UL, 5000UL, 2600);
    }
  } else {
    bursts_remaining = 0;
    if (tone_running) stopTone();
    buzzer_pin_off();
  }

  buzzerSchedulerRun();
}

const uint8_t digit_font[10][5] PROGMEM = {
  {0x3E, 0x51, 0x49, 0x45, 0x3E},
  {0x00, 0x42, 0x7F, 0x40, 0x00},
  {0x42, 0x61, 0x51, 0x49, 0x46},
  {0x21, 0x41, 0x45, 0x4B, 0x31},
  {0x18, 0x14, 0x12, 0x7F, 0x10},
  {0x27, 0x45, 0x45, 0x45, 0x39},
  {0x3C, 0x4A, 0x49, 0x49, 0x30},
  {0x01, 0x71, 0x09, 0x05, 0x03},
  {0x36, 0x49, 0x49, 0x49, 0x36},
  {0x06, 0x49, 0x49, 0x29, 0x1E}
};

const uint8_t letter_font[26][5] PROGMEM = {
  {0x7E, 0x11, 0x11, 0x11, 0x7E},
  {0x7F, 0x49, 0x49, 0x49, 0x36},
  {0x3E, 0x41, 0x41, 0x41, 0x22},
  {0x7F, 0x41, 0x41, 0x41, 0x3E},
  {0x7F, 0x49, 0x49, 0x49, 0x41},
  {0x7F, 0x09, 0x09, 0x09, 0x01},
  {0x3E, 0x41, 0x41, 0x49, 0x3A},
  {0x7F, 0x08, 0x08, 0x08, 0x7F},
  {0x00, 0x41, 0x7F, 0x41, 0x00},
  {0x20, 0x40, 0x41, 0x3F, 0x01},
  {0x7F, 0x08, 0x14, 0x22, 0x41},
  {0x7F, 0x40, 0x40, 0x40, 0x40},
  {0x7F, 0x02, 0x0C, 0x02, 0x7F},
  {0x7F, 0x04, 0x08, 0x10, 0x7F},
  {0x3E, 0x41, 0x41, 0x41, 0x3E},
  {0x7F, 0x09, 0x09, 0x09, 0x06},
  {0x3E, 0x41, 0x51, 0x21, 0x5E},
  {0x7F, 0x09, 0x19, 0x29, 0x46},
  {0x26, 0x49, 0x49, 0x49, 0x32},
  {0x01, 0x01, 0x7F, 0x01, 0x01},
  {0x3F, 0x40, 0x40, 0x40, 0x3F},
  {0x1F, 0x20, 0x40, 0x20, 0x1F},
  {0x3F, 0x40, 0x38, 0x40, 0x3F},
  {0x63, 0x14, 0x08, 0x14, 0x63},
  {0x07, 0x08, 0x70, 0x08, 0x07},
  {0x61, 0x51, 0x49, 0x45, 0x43}
};

void send_cmd(uint8_t cmd) {
  TinyI2C.start(OLED_ADDR, 0);
  TinyI2C.write(0x00);
  TinyI2C.write(cmd);
  TinyI2C.stop();
}

void init_oled_uv() {
  TinyI2C.start(OLED_ADDR, 0);
  TinyI2C.write(0x00);
  for (uint8_t i = 0; i < sizeof(Init); i++) {
    TinyI2C.write(pgm_read_byte(&Init[i]));
  }
  TinyI2C.stop();
}

void clear_display_uv() {
  for (uint8_t page = 0; page < 8; page++) {
    send_cmd(0xB0 + page);
    send_cmd(0x00);
    send_cmd(0x10);
    TinyI2C.start(OLED_ADDR, 0);
    TinyI2C.write(0x40);
    for (uint8_t i = 0; i < 128; i++) {
      TinyI2C.write(0x00);
    }
    TinyI2C.stop();
  }
}

void display_large_uv(int uv) {
  int d1 = uv / 10;
  int d2 = uv % 10;
  uint8_t start_col = (128 - 22) / 2;
  for (uint8_t p = 2; p < 4; p++) {
    send_cmd(0xB0 + p);
    send_cmd(start_col & 0x0F);
    send_cmd(0x10 + (start_col >> 4));
    TinyI2C.start(OLED_ADDR, 0);
    TinyI2C.write(0x40);
    for (int i = 0; i < 5; i++) {
      uint8_t col = pgm_read_byte(&digit_font[d1][i]);
      uint8_t scaled = 0;
      int shift = (p - 2) ? 4 : 0;
      for (int k = 0; k < 4; k++) {
        uint8_t bit = (col >> (k + shift)) & 1;
        scaled |= (bit << (2 * k)) | (bit << (2 * k + 1));
      }
      TinyI2C.write(scaled);
      TinyI2C.write(scaled);
    }
    TinyI2C.write(0x00);
    TinyI2C.write(0x00);
    for (int i = 0; i < 5; i++) {
      uint8_t col = pgm_read_byte(&digit_font[d2][i]);
      uint8_t scaled = 0;
      int shift = (p - 2) ? 4 : 0;
      for (int k = 0; k < 4; k++) {
        uint8_t bit = (col >> (k + shift)) & 1;
        scaled |= (bit << (2 * k)) | (bit << (2 * k + 1));
      }
      TinyI2C.write(scaled);
      TinyI2C.write(scaled);
    }
    TinyI2C.stop();
  }
}

void display_text(uint8_t page, uint8_t start_col, const char *str) {
  send_cmd(0xB0 + page);
  send_cmd(start_col & 0x0F);
  send_cmd(0x10 + (start_col >> 4));
  TinyI2C.start(OLED_ADDR, 0);
  TinyI2C.write(0x40);
  while (*str) {
    char c = *str++;
    if (c >= '0' && c <= '9') {
      uint8_t idx = c - '0';
      for (int i = 0; i < 5; i++) {
        TinyI2C.write(pgm_read_byte(&digit_font[idx][i]));
      }
      TinyI2C.write(0x00);
    } else if (c >= 'A' && c <= 'Z') {
      uint8_t idx = c - 'A';
      for (int i = 0; i < 5; i++) {
        TinyI2C.write(pgm_read_byte(&letter_font[idx][i]));
      }
      TinyI2C.write(0x00);
    } else {
      for (int i = 0; i < 6; i++) TinyI2C.write(0x00);
    }
  }
  TinyI2C.stop();
}

void setup_uv() {
  pinMode(BUZZ_PIN, OUTPUT);
  TinyI2C.init();
  init_oled_uv();
  clear_display_uv();
  const char *heading = "UV LEVEL";
  uint8_t heading_len = strlen(heading) * 6;
  uint8_t heading_start_col = (128 - heading_len) / 2;
  display_text(0, heading_start_col, heading);
  analogReference(INTERNAL);
  delay(5);
}

void loop_uv() {
  int adc = analogRead(A2);
  int uv = (adc * 11L) / 1023;
  const char *severity;
  if (uv <= 2) severity = "LOW";
  else if (uv <= 5) severity = "MODERATE";
  else if (uv <= 7) {
    severity = "HIGH";
  } 
  else if (uv <= 10) {
    severity = "VERY HIGH";
    digitalWrite(BUZZ_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZ_PIN, LOW);
  }
  else {
    severity = "EXTREME";
    digitalWrite(BUZZ_PIN, HIGH);
    delay(2000);
    digitalWrite(BUZZ_PIN, LOW);
  }
  for (uint8_t page = 2; page < 8; page++) {
    send_cmd(0xB0 + page);
    send_cmd(0x00);
    send_cmd(0x10);
    TinyI2C.start(OLED_ADDR, 0);
    TinyI2C.write(0x40);
    for (uint8_t i = 0; i < 128; i++) {
      TinyI2C.write(0x00);
    }
    TinyI2C.stop();
  }
  display_large_uv(uv);
  uint8_t len = strlen(severity) * 6;
  uint8_t start_col = (128 - len) / 2;
  display_text(5, start_col, severity);
  delay(1000);
}

void setup() {
  uint8_t v = EEPROM.read(EEPROM_ADDR_MODE);
  currentMode = v & 0x01;
  uint8_t newMode = currentMode ^ 0x01;
  EEPROM.update(EEPROM_ADDR_MODE, newMode);
  currentMode = newMode;
  if (currentMode == 0) {
    setup_waveform();
  } else {
    setup_uv();
  }
}

void loop() {
  if (currentMode == 0) {
    loop_waveform();
  } else {
    loop_uv();
  }
}
