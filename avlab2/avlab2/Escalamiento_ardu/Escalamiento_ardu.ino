const int IN1 = 7;
const int IN2 = 8;
const int ENA = 9;

int comando = 0;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  Serial.begin(9600);

  Serial.println("Ingrese un valor entre -100 y 100:");
}

void loop() {

  if (Serial.available() > 0) {

    comando = Serial.parseInt();

    if (comando >= -100 && comando <= 100) {

      int pwm = map(abs(comando), 0, 100, 0, 255);

      if (comando > 0) {

        // Giro en un sentido
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);

      } else if (comando < 0) {

        // Giro contrario
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);

      } else {

        // Detener
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
      }

      analogWrite(ENA, pwm);

      Serial.print("Comando: ");
      Serial.print(comando);

      Serial.print("  PWM: ");
      Serial.println(pwm);

    } else {

      Serial.println("Valor invalido. Use -100 a 100.");
    }

    while (Serial.available() > 0) {
      Serial.read();
    }

    Serial.println("Ingrese otro valor:");
  }
}