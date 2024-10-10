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
            conceptImages: ["test.jpeg", "test.jpeg", "test.jpeg"],
            video: "interaction-video1.mp4",
            arduinoCode: `// Arduino code for Project 1\nvoid setup() {}\nvoid loop() {}`,
            circuitSchematic: "a1_circuit.png",
            references: ["https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/#h-mpu6050-arduino-code", "https://www.youtube.com/watch?v=M9lZ5Qy5S2s"]
        },
        3: {
            title: "/aarij/projects/<span class='highlight'>[project title]</span>",
            highlight: "[project title]",
            description: "Lorem ipsum dolor sit amet consectetur adipisicing elit. Aperiam animi dolor fuga tempore maiores voluptates laborum quos earum expedita delectus esse, consectetur laboriosam numquam quae accusantium cum culpa repudiandae voluptate!Lorem ipsum dolor sit amet consectetur adipisicing elit. Aperiam animi dolor fuga tempore maiores voluptates laborum quos earum expedita delectus esse, consectetur laboriosam numquam quae accusantium cum culpa repudiandae voluptate!",
            conceptImages: ["test.jpeg", "test.jpeg", "test.jpeg"],
            video: "interaction-video1.mp4",
            arduinoCode: `// Arduino code for Project 1\nvoid setup() {}\nvoid loop() {}`,
            circuitSchematic: "circuit.png",
            references: ["reference1.pdf", "reference2.pdf"]
        },
        4: {
            title: "/aarij/projects/<span class='highlight'>[project title]</span>",
            highlight: "[project title]",
            description: "Lorem ipsum dolor sit amet consectetur adipisicing elit. Aperiam animi dolor fuga tempore maiores voluptates laborum quos earum expedita delectus esse, consectetur laboriosam numquam quae accusantium cum culpa repudiandae voluptate!Lorem ipsum dolor sit amet consectetur adipisicing elit. Aperiam animi dolor fuga tempore maiores voluptates laborum quos earum expedita delectus esse, consectetur laboriosam numquam quae accusantium cum culpa repudiandae voluptate!",
            conceptImages: ["test.jpeg", "test.jpeg", "test.jpeg"],
            video: "interaction-video1.mp4",
            arduinoCode: `// Arduino code for Project 1\nvoid setup() {}\nvoid loop() {}`,
            circuitSchematic: "circuit.png",
            references: ["reference1.pdf", "reference2.pdf"]
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
