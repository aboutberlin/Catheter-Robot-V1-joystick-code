// Teensy 4.1 + Joystick Control
// VRx -> 17 (A3) (Analog Input)
// VRy -> 16 (A2) (Analog Input)
// SW  -> 9  (Digital Input, using INPUT_PULLUP)

// Define pins
#define JOYSTICK_X_PIN 17  // VRx -> A3 (Analog)
#define JOYSTICK_Y_PIN 16  // VRy -> A2 (Analog)
#define JOYSTICK_SW_PIN 9  // SW  -> Digital Pin 9 (Previously was A1)

// Mode and debounce variables
int mode = 1;
bool lastSwitchState = HIGH;
bool switchPressed = false;  // To ensure single press detection
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // Debounce delay in ms

void setup() {
  Serial.begin(115200);
  
  // Set up joystick button with internal pull-up
  pinMode(JOYSTICK_SW_PIN, INPUT_PULLUP);
}

void loop() {
  // Read analog values
  int xVal = analogRead(JOYSTICK_X_PIN);
  int yVal = analogRead(JOYSTICK_Y_PIN);
  
  // Read switch state (Now from GPIO 9)
  bool swState = digitalRead(JOYSTICK_SW_PIN);

  // Debounce logic for button press
  if (swState == LOW && !switchPressed) {  // Detect new press only
    switchPressed = true;
    mode++;
    if (mode > 3) mode = 1;
    Serial.print("Mode switched to: ");
    Serial.println(mode);
  }
  if (swState == HIGH) {
    switchPressed = false;  // Reset state when button is released
  }

  // Adjusted thresholds based on your joystick behavior
  String direction = "Center";
  
  if (xVal > 950) direction = "Up";     // X high (~1023)
  else if (xVal < 50) direction = "Down";  // X low (~0)
  else if (yVal < 50) direction = "Left";  // Y low (~0)
  else if (yVal > 950) direction = "Right"; // Y high (~1023)

  // Print output
  Serial.print("X=");
  Serial.print(xVal);
  Serial.print("  Y=");
  Serial.print(yVal);
  Serial.print("  Direction=");
  Serial.print(direction);
  Serial.print("  Current Mode=");
  Serial.println(mode);

  delay(100); // Avoid screen flooding
}
