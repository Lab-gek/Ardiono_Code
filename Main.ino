
#include <Wire.h>
#include <DFRobot_RGBLCD1602.h>

// Function declarations
void displayReadyMessage();
void enterSetMode();
void checkButtons();
void updateThresholdDisplay();
void handleDistanceMeasurement();
void updateLCD();
void checkConfirmButton();
void confirmThreshold();
void measureDistance();
void updateLEDState();

// Pin definitions
const int buttonPinSet = 13;
const int buttonPinConfirm = 12;
const int ledPin = 10;
const int trigPin = 8;
const int echoPin = 9;
const int potPin = A0;

// Variable declarations
int lastButtonStateSet = HIGH;
int lastButtonStateConfirm = HIGH;
long duration;
int distance;
int thresholdDistance = 0;
int confirmedThresholdDistance = 0;

DFRobot_RGBLCD1602 lcd(0x3F, 16, 2);  // I2C address (0x3F is common, but it may vary)

unsigned long lastLCDUpdate = 0;
unsigned long lastSensorRead = 0;
const unsigned long lcdUpdateInterval = 1000;
const unsigned long sensorReadInterval = 100;

bool isBlocked = false;

void setup() {
  Serial.begin(9600);

  // Set pin modes
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  pinMode(buttonPinSet, INPUT_PULLUP);
  pinMode(buttonPinConfirm, INPUT_PULLUP);

  // Initialize I2C and LCD
  Wire.begin();  // Initialize I2C communication
  lcd.init();   // Initialize LCD
  lcd.setRGB(0, 0, 255);  // Optional: Set the initial backlight color (blue here)
  
  displayReadyMessage();
  enterSetMode();
}

void loop() {
  checkButtons();

  if (isBlocked) {
    updateThresholdDisplay();
  } else {
    handleDistanceMeasurement();
    updateLCD();
  }
}

void checkButtons() {
  checkConfirmButton();
}

void checkConfirmButton() {
  int readingConfirm = digitalRead(buttonPinConfirm);

  if (readingConfirm == LOW && lastButtonStateConfirm == HIGH) {
    if (digitalRead(buttonPinConfirm) == LOW) {
      confirmThreshold();
    }
  }

  lastButtonStateConfirm = readingConfirm;
}

void enterSetMode() {
  Serial.println("Mode Set");
  isBlocked = true;
  Serial.print("isBlocked is now: ");
  Serial.println(isBlocked);
  lcd.clear();
  lcd.print("Set Mode Active");
  lcd.setCursor(0, 1);
  lcd.print("Afstand set: ");
}

void confirmThreshold() {
  if (isBlocked) {
    thresholdDistance = map(analogRead(potPin), 0, 1023, 10, 200);
    confirmedThresholdDistance = thresholdDistance;
    Serial.print("Confirmed threshold distance: ");
    Serial.println(confirmedThresholdDistance);
    isBlocked = false;
    displayReadyMessage();
  } else {
    isBlocked = true;
    enterSetMode();
  }
}

void updateThresholdDisplay() {
  // Update LCD with threshold distance
  thresholdDistance = map(analogRead(potPin), 0, 1023, 10, 200);
  lcd.setCursor(0, 1);
  lcd.print("Afstand: ");
  lcd.print(thresholdDistance);
  lcd.print(" cm    ");
}

void handleDistanceMeasurement() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastSensorRead >= sensorReadInterval) {
    lastSensorRead = currentMillis;
    measureDistance();
    updateLEDState();
  }
}

void measureDistance() {
  // Ensure the trigger pin is low for a short period
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Send a 10 microsecond pulse to trigger the sensor
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echo pin and calculate the distance
  duration = pulseIn(echoPin, HIGH);
  distance = duration / 58.2; // Convert the time into distance (in cm)
  
  Serial.print("Measured distance: ");
  Serial.println(distance);
}

void updateLEDState() {
  if (distance >= confirmedThresholdDistance) {
    digitalWrite(ledPin, HIGH);
    Serial.println("LEDS AAN - open");
  } else {
    digitalWrite(ledPin, LOW);
    Serial.println("LEDS UIT - Gesloten");
  }
}

void updateLCD() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastLCDUpdate >= lcdUpdateInterval) {
    lastLCDUpdate = currentMillis;
    displayReadyMessage();
  }
}

void displayReadyMessage() {
  lcd.clear(); // Clear the LCD before printing new messages
  lcd.print("Kastje");
  lcd.setCursor(0, 1);
  lcd.print("Afstand: ");
  lcd.print(confirmedThresholdDistance);
  lcd.print(" cm    ");
}
