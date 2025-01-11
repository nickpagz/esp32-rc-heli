#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Motor control pins
#define MAIN_ROTOR1_PIN 3
#define MAIN_ROTOR2_PIN 4
#define TAIL_ROTOR_PIN1 5
#define TAIL_ROTOR_PIN2 6

// LED Pin
#define LED_PIN 8

// Motor states
bool motorsEnabled = false;
int mainRotorSpeed = 0;
int tailRotorSpeed = 0;

// Deadband tolerance for stick drift
const int DEADZONE = 50;

// LED state tracking
bool isControllerConnected = false;

// Define yaw offset
int yawOffset = 0;
const int YAW_OFFSET_STEP = 1;  // Step size for adjusting yaw offset

// This callback gets called any time a new gamepad is connected.
void onConnectedController(ControllerPtr ctl) {
    isControllerConnected = true;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
            myControllers[i] = ctl;
            break;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    isControllerConnected = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            break;
        }
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, buttons: 0x%04x, axis L: %4d, %4d, throttle: %4d\n",
        ctl->index(), ctl->buttons(), ctl->axisX(), ctl->axisY(), ctl->throttle());
}

void processGamepad(ControllerPtr ctl) {
    // Handle yaw adjustment buttons
    if (ctl->buttons() & 0x0020) {  // Right button pressed
        yawOffset += YAW_OFFSET_STEP;
        Serial.printf("Yaw offset increased: %d\n", yawOffset);
    }

    if (ctl->buttons() & 0x0010) {  // Left button pressed
        yawOffset -= YAW_OFFSET_STEP;
        Serial.printf("Yaw offset decreased: %d\n", yawOffset);
    }

    if (ctl->a()) {
        motorsEnabled = true;
        Serial.println("Motors enabled.");
    }

    if (ctl->b()) {
        if (abs(ctl->axisX()) < DEADZONE && abs(ctl->axisY()) < DEADZONE && ctl->throttle() < DEADZONE) {
            motorsEnabled = false;
            Serial.println("Motors disabled.");
        }
    }

    if (motorsEnabled) {
        int throttle = ctl->throttle();
        mainRotorSpeed = map(throttle, 0, 1023, 0, 255);

        // Apply yaw offset
        int yaw = ctl->axisX() - yawOffset;
        int rotor1Speed = mainRotorSpeed;  // Default
        int rotor2Speed = mainRotorSpeed;  // Default

        if (abs(yaw) > DEADZONE) {
            int yawAdjustment = map(yaw, -512, 512, -50, 50);
            rotor1Speed = constrain(mainRotorSpeed - yawAdjustment, 0, 255);
            rotor2Speed = constrain(mainRotorSpeed + yawAdjustment, 0, 255);
        }

        // Write motor speeds
        analogWrite(MAIN_ROTOR1_PIN, rotor1Speed);
        analogWrite(MAIN_ROTOR2_PIN, rotor2Speed);

        // Process pitch for tail rotor
        int pitch = ctl->axisY();
        if (abs(pitch) > DEADZONE) {
            tailRotorSpeed = map(abs(pitch), DEADZONE, 512, 0, 255);
            if (pitch > 0) {
                analogWrite(TAIL_ROTOR_PIN1, tailRotorSpeed);
                analogWrite(TAIL_ROTOR_PIN2, 0);
            } else {
                analogWrite(TAIL_ROTOR_PIN1, 0);
                analogWrite(TAIL_ROTOR_PIN2, tailRotorSpeed);
            }
        } else {
            analogWrite(TAIL_ROTOR_PIN1, 0);
            analogWrite(TAIL_ROTOR_PIN2, 0);
        }

        // Debug actual motor outputs
        Serial.printf("Rotor1 PWM: %d | Rotor2 PWM: %d | Tail Rotor PWM: %d | Yaw Offset: %d\n", rotor1Speed, rotor2Speed, tailRotorSpeed, yawOffset);
    } else {
        // Disable all motors
        analogWrite(MAIN_ROTOR1_PIN, 0);
        analogWrite(MAIN_ROTOR2_PIN, 0);
        digitalWrite(TAIL_ROTOR_PIN1, 0);
        digitalWrite(TAIL_ROTOR_PIN2, 0);
    }
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);

    pinMode(MAIN_ROTOR1_PIN, OUTPUT);
    pinMode(MAIN_ROTOR2_PIN, OUTPUT);
    pinMode(TAIL_ROTOR_PIN1, OUTPUT);
    pinMode(TAIL_ROTOR_PIN2, OUTPUT);

    // Set motor control pins to LOW at startup
    analogWrite(MAIN_ROTOR1_PIN, 0);
    analogWrite(MAIN_ROTOR2_PIN, 0);
    analogWrite(TAIL_ROTOR_PIN1, 0);
    analogWrite(TAIL_ROTOR_PIN2, 0);

    // Setup LED pin
    pinMode(LED_PIN, OUTPUT);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

        // Handle LED state
    if (isControllerConnected) {
        digitalWrite(LED_PIN, LOW);  // Solid ON when connected
    } else {
        static unsigned long lastToggle = 0;
        static bool ledState = HIGH;

        if (millis() - lastToggle > 500) {  // Flash every 500ms
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState);
            lastToggle = millis();
        }
    }

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    //     vTaskDelay(1);
    delay(20);
}
