#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>   // Universal Telegram Bot Library
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>

// ============================
// ======= CONFIGURATION =======
// ============================
#define DEEP_SLEEP_TIMER false
#define DEEP_SLEEP_EXT0 true
// Replace with your network credentials
const char* ssid = "ssid";
const char* password = "pass";

// Initialize Telegram BOT
#define BOTtoken "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"  // your Bot Token (Get from Botfather)
#define CHAT_ID "xxxxxxxxxxxxx"

// Initialize Telegram Bot with secure client
WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

// Logging constants and variables
#define MAX_LOGS  100
const unsigned long LOG_RETENTION_PERIOD = 86400000; // 24 hours in milliseconds

struct LogEntry {
    unsigned long timestamp;
    int distance;
    bool confirmed;
    byte sensorStatus;
};

// Initialize Preferences for non-volatile storage
Preferences preferences;

const char* PREF_NAMESPACE = "logStorage";
const char* INDEX_KEY = "currentLogIndex";

// Current log index
int currentLogIndex = 0;

// Pins Definitions
#define PIR_SENSOR  GPIO_NUM_33    // GPIO pin for PIR sensor (changed to 33)
#define TRIG_PIN    12    // GPIO pin for Ultrasonic TRIG
#define ECHO_PIN    14    // GPIO pin for Ultrasonic ECHO
#define BUZZER_PIN  13    // GPIO pin for Buzzer
#define LED_PIN     2     // GPIO pin for LED

// Variables
volatile bool motionDetected = false;
volatile unsigned long lastDetectionTime = 0;
const unsigned long ALERT_DURATION = 5000;      // 5 seconds
const unsigned long PIR_CHECK_INTERVAL = 60000; // 60 seconds
const unsigned long MOTION_VERIFY_DELAY = 100;   // 100 ms
const int DETECTION_DISTANCE = 300;              // cm
volatile unsigned long detectionCount = 0;
bool pirSensorFunctional = true;
unsigned long lastPIRCheck = 0;

// Telegram Bot Variables
int botRequestDelay = 1000;  // Time between bot requests in milliseconds
unsigned long lastTimeBotRan;

// ============================
// ======= FUNCTION PROTOS =====
// ============================

void initializeLogging();
void saveLogEntry(bool motionConfirmed, int measuredDistance);
void printAllLogs();
void cleanOldLogs();
void checkSerialCommands();
bool checkPIRSensorFunctionality();
bool verifyMotion();
void handleMotionDetection();
void IRAM_ATTR motionInterrupt();
void sendTelegramMessage(String message);
void prepareLightSleep();
long getUltrasonicDistance();

// ============================
// ========= SETUP =========
// ============================

void setup() {
    Serial.begin(115200);
    Serial.println(F("System initializing..."));
    
    // Initialize GPIO pins
    pinMode(PIR_SENSOR, INPUT_PULLUP);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    
    digitalWrite(TRIG_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    
    // Validate PIR_SENSOR pin for wakeup
    if (!esp_sleep_is_valid_wakeup_gpio((gpio_num_t)PIR_SENSOR)) {
        Serial.printf("GPIO %d is not a valid wakeup source for Light Sleep.\n", PIR_SENSOR);
        Serial.println(F("Please connect PIR sensor to a valid GPIO."));
    } else {
        Serial.printf("GPIO %d is a valid wakeup source for Light Sleep.\n", PIR_SENSOR);
    }
    
    // Attach interrupt to PIR sensor
    attachInterrupt(digitalPinToInterrupt(PIR_SENSOR), motionInterrupt, RISING);
    
    // Initialize Preferences [ROM]
    preferences.begin(PREF_NAMESPACE, false);
    
    initializeLogging();
    // Clean old logs immediately after initialization
    cleanOldLogs();
    
    // Connect to Wi-Fi
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    Serial.print(F("Connecting to WiFi"));
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(F("."));
    }
    Serial.println(); Serial.print(F("Connected to WiFi. IP Address: "));
    Serial.println(WiFi.localIP());
    
    // Setup secure client for Telegram
    client.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org

    // Initialize bot timing
    lastTimeBotRan = millis();
    
    Serial.println(F("System ready and configured"));
    delay(2000);
    // Enter Light Sleep initially
    prepareLightSleep();
}

// ============================
// ========= LOOP =========
// ============================

void loop() {
    // Handle Telegram Bot Updates
    if (millis() > lastTimeBotRan + botRequestDelay)  {
        int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
        while(numNewMessages) {
            Serial.println(F("Got response"));
            handleNewMessages(numNewMessages);
            numNewMessages = bot.getUpdates(bot.last_message_received + 1);
        }
        lastTimeBotRan = millis();
    }
    
    // Check PIR sensor functionality at defined intervals
    if (millis() - lastPIRCheck > PIR_CHECK_INTERVAL) {
        pirSensorFunctional = checkPIRSensorFunctionality();
        lastPIRCheck = millis();
        
        if (!pirSensorFunctional) {
            Serial.println(F("Switching to Ultrasonic-only detection"));
        }
    }
    
    // Handle motion detection
    if (motionDetected) {
        Serial.println(F("Motion detected! Waking up..."));
        handleMotionDetection();
        motionDetected = false;
    }
    
    // Handle serial commands (if any)
    checkSerialCommands();
    
    // Enter Light Sleep when idle
    prepareLightSleep();
    
    // Short delay to prevent overwhelming the loop
    delay(10);
}

// ============================
// ======= FUNCTION DEFINITIONS =======
// ============================

// Initialize Logging: Reset index and clear logs
void initializeLogging() {
    // Reset currentLogIndex to 0
    currentLogIndex = 0;
    preferences.putUInt(INDEX_KEY, currentLogIndex);
    
    // Clear existing log entries
    for (int i = 0; i < MAX_LOGS; i++) {
        String key = "log_" + String(i);
        preferences.remove(key.c_str());
    }
    
    Serial.println(F("Logging system initialized - logs cleared and index reset to 0"));
}

// Save a log entry to non-volatile storage
void saveLogEntry(bool motionConfirmed, int measuredDistance) {
    LogEntry entry;
    entry.timestamp = millis();
    entry.distance = measuredDistance;
    entry.confirmed = motionConfirmed;
    entry.sensorStatus = pirSensorFunctional ? 1 : 0;

    // Create a unique key for each log entry
    String key = "log_" + String(currentLogIndex);

    // Serialize LogEntry to a string
    String entryStr = String(entry.timestamp) + "," + 
                      String(entry.distance) + "," + 
                      String(entry.confirmed) + "," + 
                      String(entry.sensorStatus);
    
    // Save serialized LogEntry to Preferences
    preferences.putString(key.c_str(), entryStr);
    
    // Update log index
    currentLogIndex = (currentLogIndex + 1) % MAX_LOGS;
    preferences.putUInt(INDEX_KEY, currentLogIndex);
    
    Serial.println(F("Log entry saved"));
}

// Print all logs via Serial (can be adapted for Telegram) - NOT USED AFTER SLEEP IMPLEMENTATION
void printAllLogs() {
    Serial.println(F("=== Log Entries ==="));
    
    for (int i = 0; i < MAX_LOGS; i++) {
        String key = "log_" + String(i);
        String entryStr = preferences.getString(key.c_str(), "");
        
        if (entryStr.length() > 0) {
            // Parse the serialized LogEntry
            unsigned long timestamp;
            int distance;
            bool confirmed;
            byte sensorStatus;
            
            int firstComma = entryStr.indexOf(',');
            int secondComma = entryStr.indexOf(',', firstComma + 1);
            int thirdComma = entryStr.indexOf(',', secondComma + 1);
            
            if (firstComma > 0 && secondComma > firstComma && thirdComma > secondComma) {
                timestamp = entryStr.substring(0, firstComma).toInt();
                distance = entryStr.substring(firstComma + 1, secondComma).toInt();
                confirmed = entryStr.substring(secondComma + 1, thirdComma).toInt();
                sensorStatus = entryStr.substring(thirdComma + 1).toInt();
                
                // Check retention period
                if ((millis() - timestamp) <= LOG_RETENTION_PERIOD) {
                    Serial.print(F("Entry #"));
                    Serial.print(i);
                    Serial.print(F(" Time: "));
                    Serial.print(timestamp);
                    Serial.print(F(" Distance: "));
                    Serial.print(distance);
                    Serial.print(F("cm Confirmed: "));
                    Serial.print(confirmed ? "Yes" : "No");
                    Serial.print(F(" PIR Status: "));
                    Serial.println(sensorStatus ? "OK" : "Fault");
                }
            }
        }
    }
    
    Serial.println(F("=== End of Logs ==="));
}

// Clean old logs that exceed the retention period
void cleanOldLogs() {
    unsigned long currentTime = millis();
    int cleanedCount = 0;
    
    Serial.println(F("Starting cleanOldLogs..."));
    
    for (int i = 0; i < MAX_LOGS; i++) {
        String key = "log_" + String(i);
        String entryStr = preferences.getString(key.c_str(), "");
        
        if (entryStr.length() > 0) {
            // Parse the serialized LogEntry
            unsigned long timestamp;
            
            int firstComma = entryStr.indexOf(',');
            int secondComma = entryStr.indexOf(',', firstComma + 1);
            int thirdComma = entryStr.indexOf(',', secondComma + 1);
            
            if (firstComma > 0 && secondComma > firstComma && thirdComma > secondComma) {
                timestamp = entryStr.substring(0, firstComma).toInt();
                
                unsigned long timeDiff = currentTime - timestamp;
                Serial.print(F("Log #"));
                Serial.print(i);
                Serial.print(F(" Timestamp: "));
                Serial.print(timestamp);
                Serial.print(F(" Current Time: "));
                Serial.print(currentTime);
                Serial.print(F(" Time Diff: "));
                Serial.println(timeDiff);
                
                // Check retention period
                if (timeDiff > LOG_RETENTION_PERIOD) {
                    // Delete the log entry
                    preferences.remove(key.c_str());
                    cleanedCount++;
                    Serial.print(F("Deleted log #"));
                    Serial.println(i);
                }
            } else {
                Serial.print(F("Malformed log entry at #"));
                Serial.println(i);
            }
        }
    }
    
    Serial.println(F("Old logs cleaned"));
    Serial.print(F("Total logs cleaned: "));
    Serial.println(cleanedCount);
}

// Handle new Telegram messages
void handleNewMessages(int numNewMessages) {
    Serial.println(F("Handling new messages"));
    Serial.println(String(numNewMessages));
    
    for (int i = 0; i < numNewMessages; i++) {
        // Chat id of the requester
        String chat_id = String(bot.messages[i].chat_id);
        if (chat_id != CHAT_ID){
            bot.sendMessage(chat_id, "Unauthorized user", "");
            continue;
        }
        
        // Print the received message
        String text = bot.messages[i].text;
        Serial.println(text);

        String from_name = bot.messages[i].from_name;

        if (text == "/start") {
            String welcome = "Welcome, " + from_name + ".\n";
            welcome += "Use the following commands to control your device.\n\n";
            welcome += "/print_logs to retrieve all logs\n";
            welcome += "/clear_logs to delete all logs\n";
            bot.sendMessage(chat_id, welcome, "");
        }

        if (text == "/print_logs") {
            String logs = "=== Log Entries ===\n";
            for (int i = 0; i < MAX_LOGS; i++) {
                String key = "log_" + String(i);
                String entryStr = preferences.getString(key.c_str(), "");
                
                if (entryStr.length() > 0) {
                    // Parse the serialized LogEntry
                    unsigned long timestamp;
                    int distance;
                    bool confirmed;
                    byte sensorStatus;
                    
                    int firstComma = entryStr.indexOf(',');
                    int secondComma = entryStr.indexOf(',', firstComma + 1);
                    int thirdComma = entryStr.indexOf(',', secondComma + 1);
                    
                    if (firstComma > 0 && secondComma > firstComma && thirdComma > secondComma) {
                        timestamp = entryStr.substring(0, firstComma).toInt();
                        distance = entryStr.substring(firstComma + 1, secondComma).toInt();
                        confirmed = entryStr.substring(secondComma + 1, thirdComma).toInt();
                        sensorStatus = entryStr.substring(thirdComma + 1).toInt();
                        
                        // Check retention period
                        if ((millis() - timestamp) <= LOG_RETENTION_PERIOD) {
                            logs += "Entry #" + String(i) + " | Time: " + String(timestamp) + 
                                    " | Distance: " + String(distance) + "cm | Confirmed: " + 
                                    String(confirmed ? "Yes" : "No") + 
                                    " | PIR Status: " + String(sensorStatus ? "OK" : "Fault") + "\n";
                        }
                    }
                }
            }
            logs += "=== End of Logs ===";
            bot.sendMessage(chat_id, logs, "");
        }
        
        if (text == "/clear_logs") {
            // Reset log index and clear logs
            initializeLogging();
            bot.sendMessage(chat_id, "All logs have been cleared.", "");
        }
    }
}

// Function to handle serial commands 'P' (Print logs) and 'C' (Clean logs)
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
            default:
                Serial.println(F("Unknown command"));
                break;
        }
    }
}

// Check PIR sensor functionality
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
        if (distance >= DETECTION_DISTANCE) {
            Serial.println(F("PIR Sensor Malfunction Detected"));
            return false;
        }
    }
    
    return true;
}

// Verify motion using sensors
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

// Handle motion detection event
void handleMotionDetection() {
    int distance = getUltrasonicDistance();
    bool isMotionConfirmed = verifyMotion();
    
    String message;
    
    if (isMotionConfirmed) {
        detectionCount++;
        Serial.print(F("! ALERT ! - Detection: "));
        Serial.println(detectionCount);
        triggerAlarm();
        saveLogEntry(true, distance);
        
        // Prepare Telegram message
        message = "! ALERT !\n";
        message += "Detection Count: " + String(detectionCount) + "\n";
        message += "Measured Distance: " + String(distance) + " cm\n";
        message += "PIR Sensor Status: " + String(pirSensorFunctional ? "OK" : "Fault");
        
        sendTelegramMessage(message);
    } else {
        Serial.print(F("Motion Detected"));
        Serial.println(F(" Not Confirmed"));
        delay(1000);
        saveLogEntry(false, distance);
        
        // Prepare Telegram message
        message = "Motion detected but not confirmed.\n";
        message += "Measured Distance: " + String(distance) + " cm\n";
        message += "PIR Sensor Status: " + String(pirSensorFunctional ? "OK" : "Fault");
        
        sendTelegramMessage(message);
    }
}

// ISR for PIR sensor motion detection
void IRAM_ATTR motionInterrupt() {
    motionDetected = true;
    lastDetectionTime = millis();
}

// Send a message via Telegram
void sendTelegramMessage(String message) {
    if (bot.sendMessage(CHAT_ID, message, "")) {
        Serial.println(F("Telegram message sent successfully."));
    } else {
        Serial.println(F("Failed to send Telegram message."));
    }
}

// Prepare ESP32 for Light Sleep
void prepareLightSleep() {
    Serial.println("Going to sleep...");
    delay(1000);
    Serial.println("3");
    delay(1000);
    Serial.println("2");
    delay(1000);
    Serial.println("1");
    delay(1000);
    #ifdef DEEP_SLEEP_TIMER  
      esp_sleep_enable_timer_wakeup(10000000);
    #endif
    #ifdef DEEP_SLEEP_EXT0 
      pinMode(PIR_SENSOR, INPUT_PULLUP);
      rtc_gpio_hold_en(PIR_SENSOR);
      esp_sleep_enable_ext0_wakeup(PIR_SENSOR, HIGH);
    #endif
    // Enter Light Sleep
    esp_light_sleep_start();
}

// Trigger alarm with buzzer and LED
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

// Get distance measurement from ultrasonic sensor
long getUltrasonicDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout after 30ms to prevent hanging
    
    if (duration == 0) {
        Serial.println(F("Ultrasonic sensor timeout."));
        return DETECTION_DISTANCE + 1; // Return a value greater than DETECTION_DISTANCE
    }
    
    long distance = duration * 0.034 / 2; // Convert to cm
    Serial.print(F("Measured Distance: "));
    Serial.print(distance);
    Serial.println(F(" cm"));
    return distance;
}
