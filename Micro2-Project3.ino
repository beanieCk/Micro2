#include <Wire.h>
#include <RTClib.h>
#include <LiquidCrystal.h>
#include <arduinoFFT.h>
#include <TimerOne.h>

// Pin Definitions
#define MOTOR_ENABLE_PIN 5
#define MOTOR_INPUT1 4
#define MOTOR_INPUT2 3
#define SOUND_SENSOR_ANALOG A0
#define BUTTON_PIN 19 // Button connected to pin 19 (interrupt 4)
#define LCD_RS 7
#define LCD_E 8
#define LCD_D4 9
#define LCD_D5 10
#define LCD_D6 11
#define LCD_D7 12

RTC_DS1307 rtc;
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

#define SAMPLES 64 // Number of audio samples for FFT
#define SAMPLING_FREQUENCY 10000 // Sampling frequency in Hz
ArduinoFFT<double> FFT = ArduinoFFT<double>();

unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[SAMPLES];
double vImag[SAMPLES];

int fanSpeed = 0; // 0 = Off, 1 = Half, 2 = 3/4, 3 = Full
bool clockwise = true;
const char *speedLevels[] = {"0", "1/2", "3/4", "Full"};
volatile bool updateDisplay = false;

volatile bool buttonPressed = false; // Flag for button press
volatile unsigned long lastButtonPress = 0; // Time of last button press

void setup() {
  pinMode(MOTOR_INPUT1, OUTPUT);
  pinMode(MOTOR_INPUT2, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(SOUND_SENSOR_ANALOG, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Configure pin 19 as input with pull-up

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

  lcd.begin(16, 2);

  if (!rtc.begin()) {
    lcd.print("RTC Error!");
    while (1);
  }
  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
  Timer1.initialize(1000000); // 1-second interrupt
  Timer1.attachInterrupt(timerISR);

  Serial.begin(9600);

  // Initial LCD Message
  lcd.clear();
  lcd.print("Initializing...");
  delay(1000);

  // Set initial fan state
  controlMotor();
  updateLCD();
}

void loop() {
  // Handle button press
  if (buttonPressed) {
    buttonPressed = false; // Reset flag

    // Toggle fan direction
    clockwise = !clockwise;
    Serial.print("Fan Direction Changed to: ");
    Serial.println(clockwise ? "Clockwise" : "Counterclockwise");

    // Update motor and LCD
    controlMotor();
    updateLCD();

    // Delay to allow motor stabilization
    delay(300);
  }

  // Sound Sampling for FFT
  for (int i = 0; i < SAMPLES; i++) {
    microseconds = micros();
    vReal[i] = analogRead(SOUND_SENSOR_ANALOG);
    vImag[i] = 0; // Imaginary part is 0 for real input
    while (micros() < (microseconds + sampling_period_us));
  }

  // FFT Analysis
  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);
  double peak = FFT.majorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

  // Match updated C4 (474.10–563.25 Hz) or A4 (669.65–684.35 Hz)
  if (peak >= 474.10 && peak <= 563.25) { // C4
    fanSpeed = min(fanSpeed + 1, 3); // Increase speed
    updateDisplay = true;
  } else if (peak >= 669.65 && peak <= 684.35) { // A4
    fanSpeed = max(fanSpeed - 1, 0); // Decrease speed
    updateDisplay = true;
  }

  // Update Fan Control
  controlMotor();

  // Update Display Every Second
  if (updateDisplay) {
    updateDisplay = false;
    updateLCD();
  }
}

void controlMotor() {
  if (fanSpeed == 0) {
    digitalWrite(MOTOR_INPUT1, LOW);
    digitalWrite(MOTOR_INPUT2, LOW);
    analogWrite(MOTOR_ENABLE_PIN, 0); // Stop motor
  } else {
    digitalWrite(MOTOR_INPUT1, clockwise ? HIGH : LOW);
    digitalWrite(MOTOR_INPUT2, clockwise ? LOW : HIGH);
    int pwmValue = (fanSpeed == 1) ? 128 : (fanSpeed == 2) ? 192 : 255;
    analogWrite(MOTOR_ENABLE_PIN, pwmValue); // Adjust speed
  }
}

void buttonISR() {
  unsigned long currentTime = millis();

  // Debounce: Ignore if the button was pressed recently
  if (currentTime - lastButtonPress > 300) {
    buttonPressed = true; // Set flag for button press
    lastButtonPress = currentTime; // Update last press time
  }
}

void updateLCD() {
  lcd.clear(); // Clear the LCD before updating

  DateTime now = rtc.now();
  lcd.setCursor(0, 0);
  lcd.print(now.hour(), DEC);
  lcd.print(":");
  lcd.print(now.minute(), DEC);
  lcd.print(":");
  lcd.print(now.second(), DEC);

  lcd.setCursor(0, 1);
  lcd.print(clockwise ? "C" : "CC");
  lcd.print(" ");
  lcd.print(speedLevels[fanSpeed]);

  // Small delay to ensure LCD stability
  delay(50);
}

void timerISR() {
  updateDisplay = true;
}
