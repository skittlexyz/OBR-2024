int sensors[2] = {A3, A1};
int motorPins[6] = {8, 9, 10, 13, 11, 12};
// Grouped Stepper Motor pins {pwm min, a pin, b pin, ...}

int minVel = 180;
int maxVel = 220;

int leftVel = 0;
int rightVel = 0;

void setup() {
    for (int i = 0; i < 6; i++) {
        pinMode(motorPins[i], OUTPUT);
    }
    pinMode(sensors[0], INPUT);
    pinMode(sensors[1], INPUT);
    Serial.begin(115200);
}

void loop() {
    leftVel  = map(analogRead(sensors[0]), 990, 1005, minVel, maxVel);
    rightVel = map(analogRead(sensors[1]), 990, 1005, minVel, maxVel);

    Serial.print("Velocidade Esquerda: "); Serial.print(leftVel);
    Serial.print(" | Velocidade Direita: "); Serial.println(rightVel);

    analogWrite(motorPins[0], leftVel);
    analogWrite(motorPins[3], rightVel);
    digitalWrite(motorPins[1], HIGH);
    digitalWrite(motorPins[2], LOW);
    digitalWrite(motorPins[4], HIGH);
    digitalWrite(motorPins[5], LOW);

    delay(150);
}