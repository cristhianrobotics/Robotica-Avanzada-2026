const int IN1 = 7;
const int IN2 = 8;
const int ENA = 9;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Velocidad máxima
  analogWrite(ENA, 255);
}

void loop() {

  // GIRO EN UN SENTIDO
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  delay(3000);

  // DETENER
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(2000);

  // GIRO EN SENTIDO CONTRARIO
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  delay(3000);

  // DETENER
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(2000);
}