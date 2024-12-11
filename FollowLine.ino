const int sensorCount = 8;


const int sensorPins[sensorCount] = { 0 };
const int leftMotroPins[3] = { 0 };
const int rightMotroPins[3] = { 0 };

int sensorData[sensorCount] = { 0 };    // Analog data
bool onLine[sensorCount] = { false };   // Digital data

float error = 0.0F, previousError = 0.0F; // Range: (-1.0F, +1.0F)
float integral = 0.0F;
float derivative = 0.0F;
float PID = 0.0F;

// Tweak
const int threshHold = 600;
float kP = 0.0F;
float kI = 0.0F;
float kD = 0.0F;

void setPinMode() {
    for (int i = 0; i < 3; i++) {
        pinMode(leftMotroPins[i], OUTPUT);
        pinMode(rightMotroPins[i], OUTPUT);
    }
    for (int i = 0; i < sensorCount; i++) {
        pinMode(sensorPins[i], OUTPUT);
    }
}

void readSensor() {
    int activeSensor = 0;
    int sensorEffect = 0;
    for (int i = 0; i < sensorCount; i++) {
        sensorData[i] = analogRead(sensorPins[i]);
        onLine[i] = sensorData[i] < threshHold;
        activeSensor += onLine[i];
        sensorEffect += onLine[i] * i;
    }
    if (activeSensor > 0) {
        previousError = error;
        error = (float) sensorEffect / (float) activeSensor / ((float) sensorCount / 2.0F) - 1.0F;
    }
    derivative = min(1.0F, max(-1.0F, (error - previousError) * kD));
    integral += min(1.0F, max(-1.0F, integral + error * kI));

    PID = error * kP + integral + derivative;
}

void drive(float PID) {
    int leftPWM = 127 + PID * 127;
    int rightPWM = 127 - PID * 127;

    digitalWrite(leftMotroPins[0], 0);
    digitalWrite(leftMotroPins[1], 1);
    digitalWrite(rightMotroPins[0], 0);
    digitalWrite(rightMotroPins[1], 1);

    analogWrite(leftMotroPins[2], leftPWM);
    analogWrite(rightMotroPins[2], rightPWM);
}

void setup() {
    setPinMode();
}

void loop() {
    readSensor();
    drive(PID);
    delay(50);
}