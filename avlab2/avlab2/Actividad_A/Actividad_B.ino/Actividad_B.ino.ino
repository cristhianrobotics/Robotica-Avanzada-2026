const int IN1 = 7;
const int IN2 = 8;
const int ENA = 9;

int pwm = 0;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  Serial.begin(9600);

  // Sentido fijo del motor
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  Serial.println("Ingrese un valor PWM entre 0 y 255:");
}

void loop() {

  if (Serial.available() > 0) {

    pwm = Serial.parseInt();

    if (pwm >= 0 && pwm <= 255) {

      analogWrite(ENA, pwm);

      Serial.print("PWM aplicado: ");
      Serial.println(pwm);

    } else {

      Serial.println("Valor invalido. Ingrese entre 0 y 255.");
    }

    // Limpiar caracteres sobrantes
    while (Serial.available() > 0) {
      Serial.read();
    }

    Serial.println("Ingrese otro valor PWM:");
  }
}