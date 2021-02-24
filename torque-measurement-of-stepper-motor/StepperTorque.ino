#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "ACS712.h"

#define stp 4
#define dir 5
#define acs A0
#define avs A1
#define rss 2
#define OLED_RESET 4

Adafruit_SSD1306 display(OLED_RESET);
ACS712 sensor(ACS712_05B, acs);

int steps = 200;
unsigned int ticks = 14;
volatile byte pulses = 0;
unsigned long timeold = 0;
float amps = 0;
float volts = 0;
float revs = 0;
float torq = 0;

void setup() {
  Serial.begin(9600);
  startDisplay();
  //introDisplay();
  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(rss, INPUT);
  attachInterrupt(0, counter, FALLING);
}

void loop() {

  //motorClockwise();
 // amps = measureCurrent();
  volts = measureVoltage();
  //revs = measureSpeed();
  //torq = calculateTorque(amps, volts, revs);
  //parametersDisplay(amps, volts, revs, torq);
  //torqueDisplay(torq);
}

float calculateTorque(float i, float v, float n) {

  float t = (60 * i * v) / (2 * 3.14 * n);

  if (t >= 1.27) t = 1.27;

  Serial.println(String("Torque = ") + t + " Nm");
  return t;
}
void motorClockwise() {
  
  digitalWrite(dir, LOW);
  for(int i = 0; i < steps; i++)
  {
    digitalWrite(stp, HIGH);
    delay(1);
    digitalWrite(stp, LOW);
    delay(1);
  }
}

void motorCounterClockwise() {
  
  digitalWrite(dir, HIGH);
  for(int i = 0; i < steps; i++)
  {
    digitalWrite(stp, HIGH);
    delay(1);
    digitalWrite(stp, LOW);
    delay(1);
  }
}

float measureCurrent() {

  float sum = 0;
  for(int i = 0; i < 10; i++)
  {
    float val = sensor.getCurrentDC();
    sum += val;
    delay(1);
  }
  float i = sum/10.0;
  i += 0.3*i;
  if (i <= 0.25) i = 0;
  Serial.println(String("Current = ") + i + " A");
  return i;
}

float measureVoltage() {

  float sum = 0;
  float vcc = 5;
  //float vcc = readVcc();
  vcc /= 1000.0;
  for(int i = 0; i < 10; i++)
  {
    float val = analogRead(avs);
    float vout = (val * vcc) / 1024.0;
    float vin = vout / (7500.0/(30000.0+7500.0)); 
    sum += vin;
    delay(1);
  }
  float v = sum/10.0;
  if(v <= 0.05) v = 0;
  Serial.println(String("Voltage = ") + v + " V");
  return v;
}

float measureSpeed() {

  unsigned int rpm = 0;
  if (millis() - timeold >= 1000){
    detachInterrupt(0);
    rpm = (60 * 1000 / ticks )/ (millis() - timeold) * pulses;
    timeold = millis();
    pulses = 0;
    Serial.println(String("Speed = ") + rpm + " rpm");
    attachInterrupt(0, counter, FALLING);
    return rpm;
    }
}

void counter()
{
    pulses++;
}

void parametersDisplay(float i, float v, float n, float t) {

  display.setCursor(0,0);
  display.setTextSize(1);
  display.println(String("Current = ") + i + " A");
  display.println(String("Voltage = ") + v + " V");
  display.println(String("Speed = ") + n + " rpm");
  display.println(String("Torque = ") + t + " Nm");
  display.display();
  delay(1);
  display.clearDisplay();
}

void torqueDisplay(float t) {
  
  display.setCursor(0,0);
  display.setTextSize(1);
  display.println("Torque:");
  display.setTextSize(2);
  display.print(t); display.println(" Nm");
  display.display();
  delay(1);
  display.clearDisplay();
}

void startDisplay() {
  
  Serial.begin(9600);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(1000);
  display.clearDisplay();

  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.setTextSize(1);
}

void introDisplay() {

  display.setCursor(0,0);
  display.setTextSize(1);
  display.println("Muhammad");
  display.setTextSize(2);
  display.println("Abdullah");
  display.display();
  delay(1000);
  display.setCursor(60,24);
  display.setTextSize(1);
  display.println("2015-EE-166");
  display.display();
  delay(1000);
  display.clearDisplay();

  display.setCursor(0,0);
  display.setTextSize(2);
  display.println("Muaaz");
  display.setTextSize(1);
  display.println("Sadiq");
  display.display();
  delay(1000);
  display.setCursor(60,24);
  display.setTextSize(1);
  display.println("2015-EE-175");
  display.display();
  delay(1000);
  display.clearDisplay();

  display.setCursor(32,0);
  display.setTextSize(1);
  display.println("EMF Project:");
  display.setTextSize(2);
  display.println("Torque");
  display.setTextSize(1);
  display.println("Measurement");
  display.display();
  delay(1000);
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.println("Of A");
  display.setTextSize(2);
  display.println("Stepper");
  display.setTextSize(1);
  display.println("Motor");
  display.display();
  delay(1000);
  display.clearDisplay();

  display.setCursor(50,0);
  display.setTextSize(4);
  display.println("3");
  display.display();
  delay(1000);
  display.clearDisplay();
  display.setCursor(50,0);
  display.setTextSize(4);
  display.println("2");
  display.display();
  delay(1000);
  display.clearDisplay();
  display.setCursor(50,0);
  display.setTextSize(4);
  display.println("1");
  display.display();
  delay(1000);
  display.clearDisplay();
  display.setCursor(35,0);
  display.setTextSize(4);
  display.println("GO!");
  display.display();
  delay(1000);
  display.clearDisplay();
}

long readVcc() {
  
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
     ADMUX = _BV(MUX5) | _BV(MUX0) ;
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA,ADSC));
 
  uint8_t low  = ADCL;
  uint8_t high = ADCH;
 
  long result = (high<<8) | low;
 
  result = 1125300L / result;
  
  return result;
}
