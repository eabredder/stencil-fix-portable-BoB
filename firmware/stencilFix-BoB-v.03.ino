// ==== Battery meter from https://github.com/thespielplatz/esp-battery-shield ====

#include <Arduino.h>
#include <TaskScheduler.h>

#include <Adafruit_NeoPixel.h>
#include <Servo.h>

// Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Sensors
#include "BatteryMeter.h"
#include <movingAvg.h>

// ---------------------- Low-pass filter ----------------------
class LowPass {
  public:
    explicit LowPass(float RCus) : _RCus(RCus) {}
    float step(float x) {
      unsigned long now = micros();
      if (_lastMicros == 0) {
        _lastMicros = now;
        _y = x;
        return _y;
      }
      unsigned long dt = now - _lastMicros; // microseconds
      _lastMicros = now;
      // Standard one-pole: alpha = dt / (RC + dt)
      float alpha = (float)dt / (_RCus + (float)dt);
      _y += alpha * (x - _y);
      return _y;
    }
    void reset(float x) {
      _y = x;
      _lastMicros = micros();
    }
  private:
    float _y = 0.0f;
    float _RCus;                 // time constant in microseconds
    unsigned long _lastMicros = 0;
};

// ---------------------- OLED ----------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

// ---------------------- Battery ----------------------
movingAvg avgBatt(22);
const int batteryPin = A1;
const float R1 = 300000.0;
const float R2 = 100000.0;
const float vmin = 9.0;
const float vmax = 12.6;
BatteryMeter* batteryMeter = new BatteryMeter(R1, R2, batteryPin, vmin, vmax);

// ---------------------- NeoPixel ----------------------
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// ----- Onboard NeoPixel power & brightness policy -----
const uint8_t BRIGHT_ARMED   = 32;  // arming animation
const uint8_t BRIGHT_NORMAL  = 24;  // default after arming
const uint8_t BRIGHT_LOW     = 10;  // low battery dim
const uint8_t BRIGHT_CRIT    = 4;   // critical battery dim/blink
const int     BATT_LOW_PCT   = 25;  // thresholds for dimming
const int     BATT_CRIT_PCT  = 10;  // thresholds for blinking

// ---------------------- Servo / ESC ----------------------
Servo servo;

// ---------------------- Pins ----------------------
const int potPin   = A2;   // potentiometer
const int servoPin = A3;   // ESC signal (QtPy M0 supports this)

// ---------------------- Filters & State ----------------------
// Original RC = 40us; for stronger smoothing try 50–150ms (e.g., 100000 us).
LowPass lowPass(100000.0f);

// Shared state
volatile int      g_rawPot       = 0;      // 0..4095
volatile int      g_servoUs      = 1100;   // clamped microseconds for ESC
volatile int      g_vacuumPct    = 0;      // 0..100
volatile int      g_battPct      = 0;      // 0..100
volatile float    g_battVoltage  = 0.0f;   // volts
volatile uint16_t g_pixelHue     = 0;      // 0..65535
volatile bool     g_escArmed     = false;  // becomes true after arming delay

// ---------- Screen Dimming ----------
unsigned long lastActivityTime = 0;
const unsigned long screenDimTimeout = 30000;  // 30 seconds
bool screenDimmed = false;

// ---------------------- Scheduler ----------------------
Scheduler runner;

// ---------- Forward declarations ----------
void taskReadInputsCb();
void taskServoCb();
void taskPixelCb();
void taskBatteryCb();
void taskDisplayCb();
void taskSerialCb();
void taskArmEscCb();

void drawBar(int x, int y, int width, int height, int pct);
void printRightAligned(const String& s, int xRight, int y);
bool blinkState(unsigned long periodMs);

// Periods — responsive but light
Task taskReadInputs (5   /*ms*/, TASK_FOREVER, &taskReadInputsCb, &runner, false);
Task taskServo      (10  /*ms*/, TASK_FOREVER, &taskServoCb     , &runner, false);
Task taskPixel      (20  /*ms*/, TASK_FOREVER, &taskPixelCb     , &runner, false);
Task taskBattery    (250 /*ms*/, TASK_FOREVER, &taskBatteryCb   , &runner, false);
Task taskDisplay    (200 /*ms*/, TASK_FOREVER, &taskDisplayCb   , &runner, false);
Task taskSerial     (1000/*ms*/, TASK_FOREVER, &taskSerialCb    , &runner, false);

// Non-blocking ESC arming (hold 1000us for 2s, then allow control)
Task taskArmEsc     (2000/*ms*/, TASK_ONCE,   &taskArmEscCb     , &runner, false);

// ---------------------- Setup ----------------------
void setup() {
  Serial.begin(115200); // SAMD21 handles higher baud nicely

  // SAMD21 ADC: 12-bit range 0..4095
  analogReadResolution(12);

  // Display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;) { /* stop here if OLED not present */ }
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.setRotation(0);
  display.display();

  // Battery smoothing
  avgBatt.begin();

  // Onboard NeoPixel (QtPy M0 has PIN_NEOPIXEL; NEOPIXEL_POWER may or may not exist)
  #if defined(NEOPIXEL_POWER)
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
  #endif
  pixels.begin();
  pixels.setBrightness(BRIGHT_NORMAL); // start conservative
  pixels.clear();
  pixels.show();

  // IO
  pinMode(potPin, INPUT);
  pinMode(batteryPin, INPUT);

  // Servo / ESC
  servo.attach(servoPin, 1100, 1900); // SAMD21 Servo lib works on this pin

  // Initialize the low-pass to the current mapped value to avoid a jump
  int pot0 = analogRead(potPin);
  int initialUs = map(pot0, 0, 4095, 1100, 1400);
  lowPass.reset(initialUs);

  // Initialize activity timer & full brightness
  lastActivityTime = millis();
  screenDimmed = false;
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(255);

  // Start tasks
  taskReadInputs.enable();
  taskServo.enable();
  taskPixel.enable();
  taskBattery.enable();
  taskDisplay.enable();
  taskSerial.enable();

  // Start non-blocking ESC arming
  taskArmEsc.enable(); // after 2000ms it will set g_escArmed = true
}

// ---------------------- Loop ----------------------
void loop() {
  runner.execute(); // cooperative dispatcher
}

// ---------------------- Helper drawing & UI functions ----------------------

// Generic compact progress bar
void drawBar(int x, int y, int width, int height, int pct) {
  pct = constrain(pct, 0, 100);
  display.drawRect(x, y, width, height, SSD1306_WHITE);
  int fillWidth = map(pct, 0, 100, 0, width - 2);
  if (fillWidth > 0) {
    display.fillRect(x + 1, y + 1, fillWidth, height - 2, SSD1306_WHITE);
  }
}

// Right-align small text (size=1) to an xRight column
void printRightAligned(const String& s, int xRight, int y) {
  // Each character at size=1 is ~6px wide in the default GFX font
  int textWidth = s.length() * 6;
  int x = xRight - textWidth + 1;  // +1 to balance outline pixels
  if (x < 0) x = 0;
  display.setCursor(x, y);
  display.print(s);
}

// Non-blocking blink state helper
bool blinkState(unsigned long periodMs) {
  return (millis() / periodMs) % 2 == 0;
}

// ---------------------- Task Callbacks ----------------------

// Sample pot, LPF, compute servo µs and vacuum %
void taskReadInputsCb() {
  int pot = analogRead(potPin);   // 12-bit on SAMD21
  g_rawPot = pot;

  // Map pot to 1100..1400us target, then low-pass
  int targetUs = map(pot, 0, 4095, 1100, 1400);
  int filtered = (int) lowPass.step((float)targetUs);

  // Original behavior: floor to 1000 if <1110
  if (filtered < 1110) filtered = 1000;

  // Clamp safe ESC range
  filtered = constrain(filtered, 1000, 1400);
  g_servoUs = filtered;

  // Vacuum percentage from 1100..1400 → 0..100
  int pct = map(filtered, 1100, 1400, 0, 100);
  g_vacuumPct = constrain(pct, 0, 100);

  // ---------- Inactivity detection for screen dimming ----------
  static int  lastPot    = -1;
  static bool lastFanOn  = false;
  bool fanOn = (g_vacuumPct > 0 && g_escArmed);

  if (pot != lastPot || fanOn != lastFanOn) {
    lastActivityTime = millis();
    if (screenDimmed) {
      // Restore brightness immediately on activity
      display.ssd1306_command(SSD1306_SETCONTRAST);
      display.ssd1306_command(255);  // full brightness
      screenDimmed = false;
    }
  }
  lastPot   = pot;
  lastFanOn = fanOn;
}

// Drive the servo/ESC (respect arming)
void taskServoCb() {
  int outUs = g_escArmed ? g_servoUs : 1000;
  servo.writeMicroseconds(outUs);
}

// Onboard NeoPixel policy:
// - Critical battery: red blink (1 Hz)
// - Not armed: breathing amber
// - Armed: hue by pot (240°..360°), brightness dimmed on low battery
void taskPixelCb() {
  uint32_t now = millis();

  // Critical battery: slow red blink (1 Hz, 500ms on/500ms off)
  if (g_battPct <= BATT_CRIT_PCT) {
    bool on = ((now / 500) % 2) == 0;
    pixels.setBrightness(BRIGHT_CRIT);
    if (on) {
      pixels.fill(pixels.Color(255, 0, 0)); // solid red
    } else {
      pixels.clear();
    }
    pixels.show();
    return;
  }

  // ESC not yet armed: amber "breathing" (triangular wave)
  if (!g_escArmed) {
    // 1500 ms triangle wave: 0..255..0
    const uint16_t period = 1500;
    uint16_t t = now % period;
    uint8_t wave = (t < period / 2)
                     ? map(t, 0, period / 2, 10, 200)
                     : map(t, period / 2, period, 200, 10);

    pixels.setBrightness(BRIGHT_ARMED);
    // Amber-ish (HSV ≈45°): RGB ~ (255, 80, 0), scaled by 'wave'
    uint8_t r = (uint8_t)(255UL * wave / 255UL);
    uint8_t g = (uint8_t)( 80UL * wave / 255UL);
    uint8_t b = 0;
    pixels.fill(pixels.Color(r, g, b));
    pixels.show();
    return;
  }

  // Armed & running: hue reflects potentiometer and brightness depends on battery
  uint8_t bright = (g_battPct <= BATT_LOW_PCT) ? BRIGHT_LOW : BRIGHT_NORMAL;
  pixels.setBrightness(bright);

  // Hue: 240°..360° → 43690..65535 in 16-bit hue space
  uint16_t hueStart = (uint16_t)(65535UL * 240UL / 360UL); // 43690
  uint16_t hue = (uint16_t)map(g_rawPot, 0, 4095, (int)hueStart, 65535);

  pixels.fill(pixels.ColorHSV(hue, 255, 255));
  pixels.show();
}

// Read/smooth battery data
void taskBatteryCb() {
  BatteryMeter::BatteryData d = batteryMeter->getBatteryData();
  g_battVoltage = d.batteryVoltage;

  int p = avgBatt.reading(d.percentage);
  g_battPct = constrain(p, 0, 100);
}

// ======= DISPLAY: Option B3 (Grid) with W1 bars (80 px wide) =======
void taskDisplayCb() {
  // ---------- Screen Dimming Logic ----------
  unsigned long now = millis();
  if (!screenDimmed && (now - lastActivityTime > screenDimTimeout)) {
    // Dim the display
    display.ssd1306_command(SSD1306_SETCONTRAST);
    display.ssd1306_command(10);  // dim level (0–255)
    screenDimmed = true;
  }

  if (screenDimmed) {
    // Minimal message while dimmed (fast exit to save cycles)
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(22, 12);
    display.print(F("Display dimmed"));
    display.display();
    return;
  }

  display.clearDisplay();
  display.setTextSize(1); // uniform, compact

  // --- Layout constants for B3 ---
  const int barX       = 20;  // left edge of both bars
  const int barWidth   = 80;  // W1 width
  const int barHeight  = 8;   // compact & uniform
  const int vacY       = 0;   // row 1 top
  const int batY       = 12;  // row 2 top
  const int rightCol   = 127; // right edge for right-aligned text

  // ---------- Row 1: Vacuum ----------
  display.setCursor(0, vacY);
  display.print(F("Vac"));
  drawBar(barX, vacY, barWidth, barHeight, g_vacuumPct);

  {
    // Right-aligned "%": e.g., "42%"
    char buf[8];
    snprintf(buf, sizeof(buf), "%d%%", g_vacuumPct);
    printRightAligned(String(buf), rightCol, vacY);
  }

  // ---------- Row 2: Battery (Blink 'Bat' when low/critical) ----------
  bool lowBatt  = (g_battPct <= 20);
  bool critBatt = (g_battPct <= 10);

  // Blink rate: 1Hz for low, 2Hz for critical
  bool showBat = true;
  if (critBatt)      showBat = blinkState(500);
  else if (lowBatt)  showBat = blinkState(1000);

  if (showBat) {
    display.setCursor(0, batY);
    display.print(F("Bat"));
  }
  drawBar(barX, batY, barWidth, barHeight, g_battPct);

  {
    // Right-aligned "%": e.g., "78%"
    char buf[8];
    snprintf(buf, sizeof(buf), "%d%%", g_battPct);
    printRightAligned(String(buf), rightCol, batY);
  }

  // ---------- Bottom row: Fan state ----------
  const bool fanOn = (g_vacuumPct > 0 && g_escArmed);
  display.setCursor(0, 24);
  display.print(F("Fan: "));
  display.print(fanOn ? F("ON") : F("OFF"));

  display.display();
}

// Serial telemetry
void taskSerialCb() {
  Serial.println(F("Potentiometer microseconds:"));
  Serial.println(g_servoUs);
  Serial.println();

  Serial.println(F("Battery %:"));
  Serial.println(g_battPct);
  Serial.println();

  Serial.println(F("Battery Voltage:"));
  Serial.println(g_battVoltage, 3);
  Serial.println();
}

// One-shot arming completion after 2s
void taskArmEscCb() {
  g_escArmed = true;
}
