// Function to get URL parameters
function getQueryParam(name) {
    const params = new URLSearchParams(window.location.search);
    return params.get(name);
}

// Function to display project details based on the query parameter
function displayProjectInfo() {
    const projectId = getQueryParam('project');

    // Example data for each project /Users/aarij/Documents/aarij-portfolio/project_script.js
    const projectData = {
        1: {
            title: "/<a href='index.html'>aarij</a>/projects/<span class='highlight'>LumiSphere</span>",
            highlight: "LumiSphere",
            description: "The LumiSphere is an interactive nightstand LED lamp that combines modern design with intuitive controls. A joystick lets you adjust the color of the light, and a press of the joystick freezes the color at any position. Pressing the joystick again in its neutral position unfreezes the color, allowing for continuous customization. Additionally, a button controls brightness levels and acts as the on/off switch, making it easy to tailor the ambiance to your preference. This lamp is a perfect blend of style and functionality for any space.",
            conceptImages: ["cd1.jpg", "cd2.jpg", "cd3.jpg"],
            video: "interaction-video1.mp4",
            arduinoCode: `// LEDs and Button
const int R1 = 11;
const int G1 = 10;
const int BL1 = 9;
const int R2 = 6;
const int G2 = 5;
const int B2 = 3;
const int BTN = 4;

// Joystick
const int xPin = A0;
const int yPin = A1;
const int SW = 2;

// Center values for the joystick
int xCenter = 512;
int yCenter = 512;    
int deadzone = 50;    

float brightness;
bool neutral_position;
int mode;
int freeze;
int freezedHue = 0.0;
int freezedSaturation = 1.0;

int SW_state = HIGH;
int last_state = HIGH;

int BTN_state = HIGH;
int last_BTN_state = HIGH;

unsigned long lastDebounceTimeSW = 0;
unsigned long lastDebounceTimeBTN = 0;
unsigned long debounceDelay = 50;

unsigned long previousMillis = 0;
unsigned long fadeInterval = 500; // for fade/flash
unsigned long flashInterval = 20; // for fade/flash

// converts HSV to RGB
// Ref: https://www.rapidtables.com/convert/color/hsv-to-rgb.html
void hsvToRgb(float h, float s, float v, int &r, int &g, int &b) {
    float c = v * s;
    float x = c * (1 - abs(fmod((h / 60.0), 2) - 1));
    float m = v - c;

    float rPrime, gPrime, bPrime;

    if (h >= 0 && h < 60) {
        rPrime = c; gPrime = x; bPrime = 0;
    } else if (h >= 60 && h < 120) {
        rPrime = x; gPrime = c; bPrime = 0;
    } else if (h >= 120 && h < 180) {
        rPrime = 0; gPrime = c; bPrime = x;
    } else if (h >= 180 && h < 240) {
        rPrime = 0; gPrime = x; bPrime = c;
    } else if (h >= 240 && h < 300) {
        rPrime = x; gPrime = 0; bPrime = c;
    } else {
        rPrime = c; gPrime = 0; bPrime = x;
    }

    r = (rPrime + m) * 255;
    g = (gPrime + m) * 255;
    b = (bPrime + m) * 255;

    // constrains between 0 and 255
    r = constrain(r, 0, 255);
    g = constrain(g, 0, 255);
    b = constrain(b, 0, 255);
}

void setup() {
  pinMode(R1, OUTPUT);  
  pinMode(G1, OUTPUT);  
  pinMode(BL1, OUTPUT);  

  pinMode(R2, OUTPUT); 
  pinMode(G2, OUTPUT);  
  pinMode(B2, OUTPUT);  

  pinMode(BTN, INPUT);
  pinMode(SW, INPUT_PULLUP);

  brightness = 0.0;
  neutral_position = true;
  mode = 0;
  freeze = false;

  Serial.begin(9600);
  // delay to let joystick drift
  delay(3000);

  // neutral pos
  xCenter = analogRead(xPin);
  yCenter = analogRead(yPin);
}


void loop() {
  int xValue = analogRead(xPin);
  int yValue = analogRead(yPin);

  int adjustedX = xValue - xCenter;
  int adjustedY = yValue - yCenter;

  // adjust for deadzone
  if (abs(adjustedX) < deadzone) {
    adjustedX = 0;
  }
  if (abs(adjustedY) < deadzone) {
    adjustedY = 0;
  }

  // map joystick X-axis to Hue (0 to 360 degrees)
  float hue = map(adjustedX, -512, 512, 0, 360);

  // map joystick Y-axis to Saturation (0-1)
  float saturation = map(adjustedY, -512, 512, 0, 100) / 100.0;
  saturation = constrain(saturation, 0.0, 1.0);

  if (hue == 180.00 && saturation == 0.50) {
    neutral_position = true;
  }
  else {
    neutral_position = false;
  }

  // Push-button to contorl Brightness
  int btnReading = digitalRead(BTN);

  if (btnReading != last_BTN_state) {
    lastDebounceTimeBTN = millis();
  }

  if ((millis() - lastDebounceTimeBTN) > debounceDelay) {
    if (btnReading != BTN_state) {
      BTN_state = btnReading;

      if (BTN_state == LOW) {
        brightness += 0.1;
        if (brightness > 1.0) {
          brightness = 0.0;
        }
        Serial.print("Brightness: ");
        Serial.println(brightness);
      }
    }
  }
  last_BTN_state = btnReading;

  // read Joystick button
  int sw_reading = digitalRead(SW);
  if (sw_reading != last_state) {
    lastDebounceTimeSW = millis();
  }

  if ((millis() - lastDebounceTimeSW) > debounceDelay) {
    if (sw_reading != SW_state) {
      SW_state = sw_reading;

      if (SW_state == LOW) {
        if (neutral_position) {
          freeze = false;
        }
        else {
          freeze = true;
        }

        if (freeze) {
            freezedHue = hue;
            freezedSaturation = saturation;
            Serial.println("color frozen");
          }
          else {
            Serial.println("color unfrozen");
          }
      }
    }
  }
  last_state = sw_reading;

  // variables to store RGB values
  int r, g, b;

   unsigned long currentMillis = millis();


  if (!freeze) {
    hsvToRgb(hue, saturation, brightness, r, g, b);
  }
  else {
    hsvToRgb(freezedHue, freezedSaturation, brightness, r, g, b);
  }




  // write rgb values
  analogWrite(R1, r);
  analogWrite(G1, g);
  analogWrite(BL1, b);

  analogWrite(R2, r);
  analogWrite(G2, g);
  analogWrite(B2, b);

  Serial.print("Mode: ");
  Serial.print(mode);
  Serial.print(" | Brightness: ");
  Serial.println(brightness);


  delay(10);
}
`,
            circuitSchematic: "a0_circuit.png",
            references: ["https://www.rapidtables.com/convert/color/hsv-to-rgb.html", "reference2.pdf"]
        },
        2: {
            title: "/<a href='index.html'>aarij</a>/projects/<span class='highlight'>DrumSense</span>",
            highlight: "DrumSense",
            description: "DrumSense is a highly portable air drum system designed for convenience, allowing you to play or practice drums anywhere, whether at home or on the go. Using two MPU-6050 sensors, DrumSense accurately tracks stick movements and translates them into real-time drum sounds. It connects to MIDI interfaces, making it compatible with digital audio workstations, and even supports headphone use, so you can practice silently without disturbing others. DrumSense offers a seamless, immersive drumming experience, perfect for musicians looking for flexibility and mobility.",
            conceptImages: ["a1_cd1.jpg", "a1_cd2.jpg", "a1_cd3.jpg"],
            video: "a1_interaction.mp4",
            arduinoCode: `
#include <Wire.h>
#include <MPU6050.h>
#include "MIDIUSB.h"

MPU6050 mpu1(0x68); 
MPU6050 mpu2(0x69); 

unsigned long prevTime1 = 0;
unsigned long prevTime2 = 0;

const int snareNote = 38;     
const int bassNote = 36;      
const int hi_hat = 42;     
const int crashNote = 49;     
const int velocity = 100;     
const float threshold = 1.50;
const float cymbalThreshold = 1.75;
bool notePlaying1 = false;
bool notePlaying2 = false;

float filteredX1 = 0.0, filteredY1 = 0.0, filteredZ1 = 0.0;
float filteredGyroX1 = 0.0, filteredGyroY1 = 0.0, filteredGyroZ1 = 0.0;
float filteredX2 = 0.0, filteredY2 = 0.0, filteredZ2 = 0.0;
float filteredGyroX2 = 0.0, filteredGyroY2 = 0.0, filteredGyroZ2 = 0.0;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  Serial.println("Initializing MPU6050 sensors...");
  Wire.begin();

  mpu1.initialize();
  mpu2.initialize();

  if (!mpu1.testConnection()) {
    Serial.println("MPU6050-1 connection failed!");
    while (1)
      ;
  } else {
    Serial.println("MPU6050-1 connected successfully.");
  }

  if (!mpu2.testConnection()) {
    Serial.println("MPU6050-2 connection failed!");
    while (1)
      ;
  } else {
    Serial.println("MPU6050-2 connected successfully.");
  }
}

void loop() {
  int16_t ax1, ay1, az1;
  int16_t gx1, gy1, gz1;

  int16_t ax2, ay2, az2;
  int16_t gx2, gy2, gz2;

  mpu1.getAcceleration(&ax1, &ay1, &az1);
  mpu1.getRotation(&gx1, &gy1, &gz1);

  mpu2.getAcceleration(&ax2, &ay2, &az2);
  mpu2.getRotation(&gx2, &gy2, &gz2);

  float accelX1 = ax1 / 16384.0;
  float accelY1 = ay1 / 16384.0;
  float accelZ1 = az1 / 16384.0;

  float accelX2 = ax2 / 16384.0;
  float accelY2 = ay2 / 16384.0;
  float accelZ2 = az2 / 16384.0;

  float gyroX1 = gx1 / 131.0;
  float gyroY1 = gy1 / 131.0;
  float gyroZ1 = gz1 / 131.0;

  float gyroX2 = gx2 / 131.0;
  float gyroY2 = gy2 / 131.0;
  float gyroZ2 = gz2 / 131.0;

  filteredGyroX1 = abs(0.3 * gyroX1 + 0.7 * filteredGyroX1);
  filteredGyroY1 = abs(0.3 * gyroY1 + 0.7 * filteredGyroY1);
  filteredGyroZ1 = abs(0.3 * gyroZ1 + 0.7 * filteredGyroZ1);

  filteredX1 = abs(0.3 * accelX1 + 0.7 * filteredX1);
  filteredY1 = 0.3 * accelY1 + 0.7 * filteredY1;
  filteredZ1 = abs(0.3 * accelZ1 + 0.7 * filteredZ1);

  filteredGyroX2 = abs(0.3 * gyroX2 + 0.7 * filteredGyroX2);
  filteredGyroY2 = abs(0.3 * gyroY2 + 0.7 * filteredGyroY2);
  filteredGyroZ2 = abs(0.3 * gyroZ2 + 0.7 * filteredGyroZ2);

  filteredX2 = abs(0.3 * accelX2 + 0.7 * filteredX2);
  filteredY2 = 0.3 * accelY2 + 0.7 * filteredY2;
  filteredZ2 = abs(0.3 * accelZ2 + 0.7 * filteredZ2);

  float accelMagnitude1 = abs(filteredY1);

  float accelMagnitude2 = abs(filteredY2);

  unsigned long currentTime = millis();


  if (filteredX1 > 0.3 && abs(filteredGyroY1) > 180.0 && accelMagnitude1 > cymbalThreshold && !notePlaying1 && currentTime - prevTime1 > 175) {
    prevTime1 = currentTime;
    notePlaying1 = true;
    sendNoteOn(hi_hat, velocity); 
  } else if (accelMagnitude1 > threshold && abs(filteredGyroY1) < 100.0 && !notePlaying1 && currentTime - prevTime1 > 175) {
    prevTime1 = currentTime;
    notePlaying1 = true;
    sendNoteOn(snareNote, velocity);
  }


  if (filteredX2 > 0.3 && abs(filteredGyroY2) > 180.0 && accelMagnitude2 > cymbalThreshold && !notePlaying2 && currentTime - prevTime2 > 175) {
    prevTime2 = currentTime;
    notePlaying2 = true;
    sendNoteOn(crashNote, velocity);
  } else if (accelMagnitude2 > threshold && abs(filteredGyroY2) < 100.0 && !notePlaying2 && currentTime - prevTime2 > 175) {
    prevTime2 = currentTime;
    notePlaying2 = true;
    sendNoteOn(bassNote, velocity);
  }

  if (accelMagnitude1 < threshold && notePlaying1) {
    notePlaying1 = false;
    sendNoteOff(snareNote, 0);
    sendNoteOff(hi_hat, 0);
  }

  if (accelMagnitude2 < threshold && notePlaying2) {
    notePlaying2 = false;
    sendNoteOff(bassNote, 0);
    sendNoteOff(crashNote, 0);
  }
}

void sendNoteOn(byte pitch, byte velocity) {
  midiEventPacket_t noteOn = {0x09, 0x90, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
  MidiUSB.flush();
}

void sendNoteOff(byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {0x08, 0x80, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
  MidiUSB.flush();
}
`,
            circuitSchematic: "a1_circuit.png",
            references: ["https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/#h-mpu6050-arduino-code", "https://www.youtube.com/watch?v=M9lZ5Qy5S2s"]
        },
        3: {
            title: "/aarij/projects/<span class='highlight'>SmartGlass</span>",
            highlight: "SmartGlass",
            description: "SmartGlass is an intelligent timing system that merges the classic charm of an hourglass with modern technology to enhance your productivity. Equipped with an Arduino, stepper motor, LCD, and keypad, SmartGlass allows you to set customizable timers with ease. For durations longer than the default 60-minute hourglass, the device automatically rotates the hourglass 180 degrees, adding extra hours as needed. Integrated with a Chrome extension, you can specify 'irrelevant' websites that might distract you during work or study sessions. Visiting these sites prompts SmartGlass to pause the timer by rotating the hourglass 90 degrees, resuming only when you close the distracting tabs. This ensures you remain accountable and focused, effectively studying or working for the full duration you've set. By blending physical and digital elements, SmartGlass offers a unique and engaging way to manage your time, making it an ideal tool for students and professionals seeking to boost their productivity.",
            conceptImages: ["a2_cd1.png", "a2_cd2.jpg", "a2_cd3.jpg"],
            video: "interaction-video1.mp4",
            arduinoCode: `// LATEST
#include <Keypad.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>

#define MotorInterfaceType 8  // For 28BYJ-48 with ULN2003

#define motorPin1  10     // IN1 on the ULN2003 driver
#define motorPin2  11     // IN2 on the ULN2003 driver
#define motorPin3  12     // IN3 on the ULN2003 driver
#define motorPin4  13     // IN4 on the ULN2003 driver

AccelStepper stepper = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);


LiquidCrystal_I2C lcd(0x27, 16, 2);

const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {5, 4, 3, 2}; // 8, 7, 6, 5
byte colPins[COLS] = {9, 8, 7, 6}; // 4, 3, 2, 1
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

const int stepsPerRevolution = 4076;

unsigned long timerDuration = 0;
unsigned long timerStartTime = 0;
unsigned long elapsedTime = 0;
bool timerRunning = false;
bool timerPaused = false;
bool timerPausedByUser = false;
String inputBuffer = ""; 

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  lcd.init();lcd.backlight();lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Enter hours:");

  stepper.setMaxSpeed(1000.0);
  stepper.setAcceleration(500.0);
}

void loop() {
  handleKeypadInput();
  handleSerialInput();
  updateTimer();
}

void handleSerialInput() {
  if (Serial.available() > 0) {
    String serialInput = Serial.readStringUntil('\n');
    Serial.println("Serial Input: " + String(serialInput));
    if (serialInput == String('p')) { // PAUSE/UNPAUSE message from browser
      if (!timerPausedByUser) {
        togglePause();
      }
    }
    else if (serialInput == 'r') { // simulates REVERSE message from browser
      Serial.println("TODO: Reverse timer as punishment");
    }
    else if (serialInput == String('d')) { // debug
      Serial.print("Timer running: ");
      Serial.println(timerRunning);
      Serial.print("Timer Paused: ");
      Serial.println(timerPaused);
      Serial.print("Elapsed: ");
      Serial.println(elapsedTime / 1000.0);
    }
  }
}

void handleKeypadInput() {
  char key = keypad.getKey();

  if (key) {
    Serial.println("Keypad Input: " + String(key));
    if (key >= '0' && key <= '9') {
      if (!timerRunning) {
        inputBuffer += key;
        lcd.setCursor(0, 1);
        lcd.print(inputBuffer + "       ");
      }
    } else if (key == '*') {
      if (!timerRunning) {
        inputBuffer = "";
        lcd.setCursor(0, 1);
        lcd.print("Input cleared   ");
        delay(1000);
        lcd.setCursor(0, 1);
        lcd.print("                ");
      }
    } else if (key == '#') {
      if (!timerRunning && inputBuffer.length() > 0) {
        startTimer(inputBuffer.toInt());
      }
    } else if (key == 'A') {
      if (timerRunning) {
        timerPausedByUser = !timerPausedByUser;
        togglePause();
      }
    } else if (key == 'C') {
      if (timerRunning && !timerPaused){
        cancelTimer();
      }
    }
  }
}

void startTimer(int hours) {
  if (hours < 1 || hours > 9) {
    lcd.setCursor(0, 1);
    lcd.print("Invalid Input   ");
    inputBuffer = "";
    delay(2000);
    lcd.setCursor(0, 1);
    lcd.print("Enter hours:    ");
    return;
  }
  timerDuration = (unsigned long)hours * 60 * 60 * 1000; // Convert hours to milliseconds
  timerStartTime = millis();
  elapsedTime = 0;
  timerRunning = true;
  timerPaused = false;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Timer started");
  lcd.setCursor(0, 1);
  lcd.print("for " + String(hours) + " hour(s)");

  inputBuffer = "";

  // Move motor to START state
  moveToStartState();
}

void cancelTimer() {
  timerRunning = false;
  timerPaused = false;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Timer Canceled");
  moveToStopState(); // Return to INITIAL state
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Enter hours:");
}

void togglePause() {
  if (timerRunning) {
    timerPaused = !timerPaused;
    if (timerPaused) {
      lcd.setCursor(0, 1);
      lcd.print("Timer Paused    ");
      moveToPauseState();
    } else {
      lcd.setCursor(0, 1);
      lcd.print("Timer Resumed   ");
      moveToStartStateFromPause();
      // Adjust the timerStartTime to account for the paused duration
      timerStartTime = millis() - elapsedTime;
    }
  }
}

void updateTimer() {
  if (timerRunning && !timerPaused) {
    elapsedTime = millis() - timerStartTime;

    // Check if timer has completed
    if (elapsedTime >= timerDuration) {
      timerRunning = false;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Timer Completed");
    }
    else {
      // Check if it's time to flip the hourglass
      unsigned long flipsNeeded = elapsedTime / (60 * 60 * 1000); // Number of hours elapsed
      static unsigned long lastFlips = 0;

      if (flipsNeeded > lastFlips) {
        lastFlips = flipsNeeded;
        // Flip the hourglass
        moveToStartState(); // Flip 180 degrees
      }
    }

  }
}

void moveToStartState() {
  int steps = stepsPerRevolution / 2; // 180 degrees
  moveStepper(steps);
}
void moveToStartStateFromPause() {
  int steps = stepsPerRevolution / 4; // 90 degrees
  moveStepper(-steps);
}

void moveToPauseState() {
  if (timerRunning) {
    int steps = stepsPerRevolution / 4; // 90 degrees
    moveStepper(steps);
  }
}

void moveToStopState() {
  int steps = stepsPerRevolution / 2; // 180 degrees
  moveStepper(steps);
}

void moveStepper(int stepsToMove) {
  stepper.move(stepsToMove);
  stepper.runToPosition();
}


`,
            circuitSchematic: "a2_circuit.png",
            references: ["https://www.yeggi.com/q/shaft+coupler/", "https://www.instructables.com/BYJ48-Stepper-Motor/"]
        },
        4: {
            title: "/aarij/projects/<span class='highlight'>PulseLink</span>",
            highlight: "PulseLink",
            description: `PulseLink is an interactive wearable project featuring two 3D-printed bracelets equipped with ESP32 microcontrollers, vibration motors, and force sensors. Using ESP-NOW for wireless communication, the bracelets create a real-time connection between users. When one person touches the force sensor on their bracelet, the other person's bracelet provides haptic feedback through a vibrating motor, fostering a sense of physical connection across distances. PulseLink seamlessly blends hardware design, wireless communication, and user experience to enable intuitive, tactile interaction.`,
            conceptImages: ["a3_cd1.jpg", "a3_cd2.jpg", "a3_cd3.jpg"],
            video: "a3_interaction.mp4",
            arduinoCode: `#include <WiFi.h>
#include <esp_now.h>

// Define pins
#define MOTOR_PIN SCK          // GPIO pin connected to the transistor base
#define FORCE_SENSOR_PIN A2  // Analog pin connected to the force sensor
const int threshold = 50;  // Adjust based on your sensor readings

uint8_t peerMAC[] = {0xcc, 0xdb, 0xa7, 0x32, 0x1a, 0x6c};
char message[250];


// Callback function to handle received messages
void onDataReceived(const esp_now_recv_info_t* recvInfo, const uint8_t* data, int dataLen) {
  Serial.print("Message received from: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", recvInfo->src_addr[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  // Process the received data
  char message[dataLen + 1];
  memcpy(message, data, dataLen);
  message[dataLen] = '\0';  // Null-terminate the message

  Serial.printf("Message: %s\n", message);
  if (strcmp(message, "ON") == 0) {
    digitalWrite(MOTOR_PIN, HIGH);
  }
  else if (strcmp(message, "OFF") == 0) {
    digitalWrite(MOTOR_PIN, LOW);
  }
}

// Callback function to handle message sending status
void onDataSent(const uint8_t* macAddr, esp_now_send_status_t status) {
  Serial.print("Last message sent to: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", macAddr[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.print(" Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}




void setup() {
  Serial.begin(115200);
  // Set up the motor control pin
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);  // Ensure motor is off at startup

  // Initialize Wi-Fi in station mode
  WiFi.mode(WIFI_STA);
  Serial.println("Wi-Fi initialized in STA mode");

   // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed!");
    return;
  }
  Serial.println("ESP-NOW initialized");

  // Register callback functions
  esp_now_register_recv_cb(onDataReceived);
  esp_now_register_send_cb(onDataSent);

  // Register the peer
  esp_now_peer_info_t peerInfo;
  esp_now_del_peer(peerMAC);
  memcpy(peerInfo.peer_addr, peerMAC, 6);
  peerInfo.channel = WiFi.channel();  // Use default Wi-Fi channel
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  Serial.println("Peer added");

}

void loop() {
  Serial.println(WiFi.macAddress());
  // Serial.println(WiFi.macAddress());
  // Read the analog value from the force sensor
  int sensorValue = analogRead(FORCE_SENSOR_PIN);

  // Check if the sensor value exceeds the threshold
  if (sensorValue > threshold) {
    const char* message = "ON";
    esp_err_t result = esp_now_send(peerMAC, (uint8_t*)message, strlen(message));

    if (result == ESP_OK) {
      Serial.println("Message sent successfully");
    } else {
      Serial.println("Error sending message");
    }
  } 
  else {
    const char* message = "OFF";
    esp_err_t result = esp_now_send(peerMAC, (uint8_t*)message, strlen(message));

    if (result == ESP_OK) {
      Serial.println("Message sent successfully");
    } else {
      Serial.println("Error sending message");
    }
  }

  delay(250);
}`,
            circuitSchematic: "a3_circuit.png",
            references: ["https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/", "https://www.youtube.com/watch?v=3hoBwa0ccys"]
        },
    };

    const project = projectData[projectId];

    if (project) {
        document.getElementById('project-title').innerHTML = `${project.title}`;
        document.getElementById('project-desc').innerHTML = project.description;

        const conceptImagesContainer = document.getElementById('concept-images-container');
        conceptImagesContainer.innerHTML = project.conceptImages.map(src => 
            `<img src="${src}" alt="Concept Image" class="concept-image">`
        ).join('');

        document.getElementById('project-video').src = project.video;

        document.getElementsByClassName('language-arduino')[0].innerHTML = project.arduinoCode;

        document.getElementById('circuit-schematic').src = project.circuitSchematic;

        const referenceMaterials = document.getElementById('reference-materials');
        referenceMaterials.innerHTML = project.references.map(ref => 
            `<li><a href="${ref}" target="_blank">${ref}</a></li>`
        ).join('');
    } else {
        document.getElementById('project-info').innerHTML = `
            <h1>Project Not Found</h1>
        `;
    }
}

// Execute the function on page load
document.addEventListener('DOMContentLoaded', displayProjectInfo);
