#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>


// LCD pin definitions
#define LCD_RS  7
#define LCD_EN  8
#define LCD_D4  9
#define LCD_D5  10
#define LCD_D6  11
#define LCD_D7  12

// Initialize LCD object
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Pin definitions
#define PIR_SENSOR  2
#define TRIG_PIN  5      
#define ECHO_PIN  6       
#define BUZZER_PIN  4     
#define LED_PIN  13      

// Variables
volatile bool motionDetected = false;
volatile unsigned long lastDetectionTime = 0;
const unsigned long ALERT_DURATION = 5000;      
const unsigned long DETECTION_THRESHOLD = 1000;  


volatile unsigned long detectionCount = 0;
unsigned long lastResetTime = 0;
const unsigned long STATS_RESET_INTERVAL = 86400000;  

// Added constants and variables
const unsigned long PIR_CHECK_INTERVAL = 60000;
#define MOTION_VERIFY_DELAY  100
#define DETECTION_DISTANCE 300 
#define DISTANCE_THRESHOLD  300
#define MIN_DISTANCE  20

bool pirSensorFunctional = true;
unsigned long lastPIRCheck = 0;

// Logging constants and variables
#define MAX_LOGS  100
#define EEPROM_START_ADDR  0
const unsigned long LOG_RETENTION_PERIOD = 86400000; // 24 hours

struct LogEntry {
  unsigned long timestamp;
  int distance;
  bool confirmed;
  byte sensorStatus;
};

#define LOG_ENTRY_SIZE  sizeof(LogEntry)
int currentLogIndex = 0;
unsigned long lastLogCleanup = 0;

//====================================MAIN CODE====================================================
void setup() {
  Serial.begin(9600);
  Serial.println(F("System initializing..."));
  
  lcd.begin(16, 2);  
  
  pinMode(PIR_SENSOR, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(PIR_SENSOR), motionInterrupt, RISING);
  
  initializeLogging();
  
  lcd.clear();
  lcd.print("System Ready");
  Serial.println(F("System ready and configured"));
  delay(2000);
  
  prepareSleep();
}

void loop() {
  if (millis() - lastPIRCheck > PIR_CHECK_INTERVAL) {
    pirSensorFunctional = checkPIRSensorFunctionality();
    lastPIRCheck = millis();
    
    if (!pirSensorFunctional) {
      Serial.println(F("Switching to Ultrasonic-only detection"));
    }
  }
  
  if (motionDetected) {
    Serial.println(F("Motion detected! Waking up..."));
    handleMotionDetection();
    motionDetected = false;
    prepareSleep();
  }
  
  checkSerialCommands();
}

//================================LOGGING FUNC========================================
void initializeLogging() {
  EEPROM.get(EEPROM_START_ADDR + (MAX_LOGS * LOG_ENTRY_SIZE), currentLogIndex);
  
  if (currentLogIndex >= MAX_LOGS) {
    currentLogIndex = 0;
    EEPROM.put(EEPROM_START_ADDR + (MAX_LOGS * LOG_ENTRY_SIZE), currentLogIndex);
  }
  
  Serial.println(F("Logging system initialized"));
}

void saveLogEntry(bool motionConfirmed, int measuredDistance) {
  LogEntry entry;
  entry.timestamp = millis();
  entry.distance = measuredDistance;
  entry.confirmed = motionConfirmed;
  entry.sensorStatus = pirSensorFunctional ? 1 : 0;

  int address = EEPROM_START_ADDR + (currentLogIndex * LOG_ENTRY_SIZE);
  EEPROM.put(address, entry);
  
  currentLogIndex = (currentLogIndex + 1) % MAX_LOGS;
  EEPROM.put(EEPROM_START_ADDR + (MAX_LOGS * LOG_ENTRY_SIZE), currentLogIndex);
  
  Serial.println(F("Log entry saved"));
}

void printAllLogs() {
  Serial.println(F("=== Log Entries ==="));
  
  for (int i = 0; i < MAX_LOGS; i++) {
    LogEntry entry;
    int address = EEPROM_START_ADDR + (i * LOG_ENTRY_SIZE);
    EEPROM.get(address, entry);
    
    if (entry.timestamp != 0xFFFFFFFF && 
        (millis() - entry.timestamp) <= LOG_RETENTION_PERIOD) {
      
      Serial.print(F("Entry #"));
      Serial.print(i);
      Serial.print(F(" Time: "));
      Serial.print(entry.timestamp);
      Serial.print(F(" Distance: "));
      Serial.print(entry.distance);
      Serial.print(F("cm Confirmed: "));
      Serial.print(entry.confirmed ? "Yes" : "No");
      Serial.print(F(" PIR Status: "));
      Serial.println(entry.sensorStatus ? "OK" : "Fault");
    }
  }
  Serial.println(F("=== End of Logs ==="));
}

void cleanOldLogs() {
  unsigned long currentTime = millis();
  
  for (int i = 0; i < MAX_LOGS; i++) {
    LogEntry entry;
    int address = EEPROM_START_ADDR + (i * LOG_ENTRY_SIZE);
    EEPROM.get(address, entry);
    
    if (entry.timestamp != 0xFFFFFFFF && 
        (currentTime - entry.timestamp) > LOG_RETENTION_PERIOD) {
      
      for (int j = 0; j < LOG_ENTRY_SIZE; j++) {
        EEPROM.write(address + j, 0xFF);
      }
    }
  }
  
  lastLogCleanup = currentTime;
  Serial.println(F("Old logs cleaned"));
}

void checkSerialCommands() {
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'P': // Print all logs
        printAllLogs();
        break;
      case 'C': // Clean old logs
        cleanOldLogs();
        break;
    }
  }
}

//==================================== SENSOR FUNC ========================================
bool checkPIRSensorFunctionality() {
  int pirReadings[5];
  int highReadings = 0;
  
  for (int i = 0; i < 5; i++) {
    pirReadings[i] = digitalRead(PIR_SENSOR);
    if (pirReadings[i] == HIGH) {
      highReadings++;
    }
    delay(200);
  }
  
  if (highReadings > 2) {
    int distance = getUltrasonicDistance();
    if (distance > DETECTION_DISTANCE) {
      Serial.println(F("PIR Sensor Malfunction Detected"));
      return false;
    }
  }
  
  return true;
}

bool verifyMotion() {
  if (!pirSensorFunctional) {
    int distances[3];
    for (int i = 0; i < 3; i++) {
      distances[i] = getUltrasonicDistance();
      delay(50);
    }
    int avgDistance = (distances[0] + distances[1] + distances[2]) / 3;
    return (avgDistance <= DETECTION_DISTANCE);
  } else {
    if (digitalRead(PIR_SENSOR) == HIGH) {
      delay(MOTION_VERIFY_DELAY);
      int distances[3];
      for (int i = 0; i < 3; i++) {
        distances[i] = getUltrasonicDistance();
        delay(50);
      }
      int avgDistance = (distances[0] + distances[1] + distances[2]) / 3;
      return (avgDistance <= DETECTION_DISTANCE);
    }
    return false;
  }
}

void handleMotionDetection() {
  lcd.display();
  int distance = getUltrasonicDistance();
  bool isMotionConfirmed = verifyMotion();
  
  if (isMotionConfirmed) {
    detectionCount++;
    Serial.print("! ALERT ! - Detection: ");
    Serial.println(detectionCount);
    triggerAlarm();
    saveLogEntry(true, distance);
  } else {
    Serial.print("Motion Detected");
    Serial.println("Not Confirmed");
    delay(1000);
    saveLogEntry(false, distance);
  }
  
  if (millis() - lastLogCleanup >= LOG_RETENTION_PERIOD) {
    cleanOldLogs();
  }
  
  lcd.noDisplay();
}

void motionInterrupt() {
  Serial.println(F("PIR INTERRUPT TRIGGERED!"));
  sleep_disable();
  motionDetected = true;
  lastDetectionTime = millis();
}

void prepareSleep() {
  // Prepare for sleep
  Serial.println("Entering sleep mode...");
  delay(100);  // Allow serial to complete transmission
  
  // Configure sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  
  // Disable ADC and other peripherals to save power
  power_adc_disable();
  power_spi_disable();
  //power_timer0_disable();
  power_timer2_disable();
  power_twi_disable();
  
  // Enter sleep mode
  sleep_mode();
  
  // System resumes here after wake-up
  sleep_disable();
  
  // Re-enable peripherals
  power_all_enable();
  
  Serial.println("Waking up from sleep mode");
}

void triggerAlarm() {
  unsigned long startTime = millis();
  while (millis() - startTime < ALERT_DURATION) {
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

long getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.034 / 2;
}