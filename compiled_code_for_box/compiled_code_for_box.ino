// Import Neopixel module (onboard LED) and display modules
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Define relevant pins for stim/thermistor readout and moving average buffer size
#define LED_PIN A0
#define POTENTIOMETER_PIN A1
#define THERMISTOR_PIN A2
#define PARADIGM_PIN A3
#define BUFFER_SIZE 10

// Define Display properties
#define SCREEN_ADDRESS 0x3D
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64

// Initialize onboard LED and create relevant variables for stimulation
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL);
unsigned long innerStartTime, outerStartTime, currentTime, update;
const unsigned int pulseWidth_c = 1e6;
const unsigned int period_c = 2e6;
const unsigned int pulseWidth_stim = 5e3;
const unsigned int period_stim = 245e3;
const unsigned int on_stim = 5e6;
const unsigned int off_stim = 20e6;
const unsigned int errorTime = 3.6e9;
int potentiometerBuffer[BUFFER_SIZE];

// Initialize SSD display
Adafruit_SSD1306 display(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire);

// Create relevant variables for thermistor
const unsigned int fixed_resistor = 100000;
const float c1 = 0.0013886299, c2 = 0.00013769741, c3 = 2.5665933e-07;
const unsigned int updateTime = 1e6;
int temperatureBuffer[BUFFER_SIZE];
float startingTemp;

void setup() {
  Serial.begin(9600);
  delay(500);

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();

  pinMode(PARADIGM_PIN, OUTPUT);
  analogReadResolution(12);
  analogWriteResolution(10);

  float startingTemp_V = 0;
  for (int writeIdx = 0; writeIdx < BUFFER_SIZE; writeIdx++) {
    startingTemp_V += analogRead(THERMISTOR_PIN);
  }
  float filtered_startingTemp_V = float(startingTemp_V)/BUFFER_SIZE;
  startingTemp = convertVoltagetoTemp(filtered_startingTemp_V);

  pixels.begin();
  innerStartTime = micros();
  outerStartTime = micros();
}

void loop() {
  currentTime = micros();
  static int writeIdx = 0;

  int potentiometerInput = analogRead(POTENTIOMETER_PIN);
  int temperatureInput = analogRead(THERMISTOR_PIN);

  potentiometerBuffer[writeIdx] = potentiometerInput;
  temperatureBuffer[writeIdx] = temperatureInput;
  writeIdx++;
  writeIdx = (writeIdx >= BUFFER_SIZE) ? 0 : writeIdx;
  long pot_sum = 0;
  long temp_sum = 0;

  for (int i = 0; i < BUFFER_SIZE; i++) {
    pot_sum += potentiometerBuffer[i];
    temp_sum += temperatureBuffer[i];
  }
  int filtered_pot = float(pot_sum)/BUFFER_SIZE;
  float filtered_temp = float(temp_sum)/BUFFER_SIZE;

  //potentiometerInput = map(potentiometerInput, 0, 4095, 0, 460); //500 => 340 -> 315 mW; 460 => 300 -> 275 mW was on 350
  filtered_pot = map(filtered_pot, 0, 4095, 0, 350);
  float output_A0 = filtered_pot * (3.3/1023);
  if (currentTime <= errorTime) {
    pixels.setPixelColor(0, pixels.Color(0, 0, 255));
    beginCallibration();
    //beginStimulation();
    analogWrite(LED_PIN, filtered_pot);
  } else {
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    beginCallibration();
    analogWrite(LED_PIN, 0);
  }
  
  //Thermistor Resistor
  float T = convertVoltagetoTemp(filtered_temp);

  if (currentTime >= update) {
    displayVoltage(output_A0);
    displayTemp(T);
    update += updateTime;
  }
}

// Functions used below:
// displayData: Show show relevant information onto SSD display
void displayVoltage(float potentiometer) {
  display.setCursor(0,0);
  display.setTextColor(WHITE, BLACK);
  display.print(F("A0: "));
  display.print(potentiometer);
  display.print(" V");
  display.display();
}

void displayTemp(float temperature) {
  display.setCursor(0,20);
  display.setTextColor(WHITE, BLACK);
  display.print(F("Curr Temp: "));
  display.print(temperature);
  display.print(" C ");
  display.setCursor(0, 40);
  display.print(F("Start Temp: "));
  display.print(startingTemp);
  display.print(" C ");
  display.display();
}

// cyleTiming: start stim paradigm with specified pulsewidth and period/frequency
void cycleTiming(int pulseWidth, int period) {
  if (currentTime - innerStartTime <= pulseWidth) {
    pixels.show();
    digitalWrite(PARADIGM_PIN, HIGH);
  } else {
    pixels.clear();
    pixels.show();
    digitalWrite(PARADIGM_PIN, LOW);
  }

  if (currentTime - innerStartTime >= period) {
    innerStartTime = micros();
  }
}

// beginStimulation: start specific stim paradigm (5 seconds on, 15 seconds off: 4 Hz; 5 ms Pulsewidth)
void beginStimulation() {
  if (currentTime - outerStartTime <= on_stim) {
    cycleTiming(pulseWidth_stim, period_stim);
  } else {
    pixels.clear();
    pixels.show();
    digitalWrite(PARADIGM_PIN, LOW);
  }

  if (currentTime - outerStartTime >= off_stim) {
    outerStartTime = micros();
  }
}

// beginCallibration: start specific stim paradigm (1 second on, 1 second off)
void beginCallibration() {
  cycleTiming(pulseWidth_c, period_c);
}

float convertVoltagetoTemp(float voltage) {
  voltage = (4095 / voltage) - 1;
  float R_th = fixed_resistor / voltage;
  float logR = log(R_th);
  float T = (1.0 / (c1 + c2*logR + c3*logR*logR*logR)) - 273.15;
  return T;
}