int irEmitterPins[] = {0, 1, 2, 3};
int irReaderPins[] = {A0, A1, A2, A3};

char irReads[4];
char irStates[4];

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 4; i++) {
    pinMode(irEmitterPins[i], OUTPUT);
    pinMode(irReaderPins[i], INPUT);
  }
}

void loop() {
  for (int i = 0; i < 4; i++) {
    if (digitalRead(irReaderPins[i])) irReads[i] = '#';
    else irReads[i] = '_';
  }

  String outputMessage = String("+---------------------+\n") +
                         "|   TCRT5000 Tester   |\n" +
                         "+----+-------+--------+\n" +
                         "| ID | Input | Output |\n" +
                         "+----+-------+--------+\n" +
                         "|  0 |     " + String(irStates[0]) + "|      " + String(irReads[0]) + "|\n" +
                         "|  1 |     " + String(irStates[1]) + "|      " + String(irReads[1]) + "|\n" +
                         "|  2 |     " + String(irStates[2]) + "|      " + String(irReads[2]) + "|\n" +
                         "|  3 |     " + String(irStates[3]) + "|      " + String(irReads[3]) + "|\n" +
                         "+----+-------+--------+";
  Serial.println(outputMessage);
  delay(100);
}