// ======================================================
// ACTIVIDAD 3.2
// CONTROL PID DE POSICION
// Arduino UNO + L298N + Encoder
// ======================================================


// ---------------- MOTOR ----------------
#define IN1 7
#define IN2 8
#define ENA 9


// ---------------- ENCODER ----------------
#define ENC_A 3
#define ENC_B 2


// ---------------- ENCODER ----------------
// Valor experimental obtenido anteriormente
const float CUENTAS_POR_REV = 925.0;


// ---------------- TIEMPO DE MUESTREO ----------------
const float dt = 0.05;       // 50 ms
const unsigned long Ts = 50;


// ---------------- ENCODER ----------------
volatile long contadorEncoder = 0;


// ---------------- POSICION ----------------
float posicion = 0.0;       // grados
float referencia = 0.0;     // grados


// ---------------- PID ----------------
float Kp = 2.0;
float Ki = 0.0;
float Kd = 0.3;

float error = 0.0;
float errorAnterior = 0.0;

float integral = 0.0;
float derivativo = 0.0;

float control = 0.0;


// ---------------- TIEMPO ----------------
unsigned long tiempoAnterior = 0;


// ======================================================
// SETUP
// ======================================================

void setup() {

  Serial.begin(9600);

  // Encoder
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(ENC_A),
    contarEncoder,
    RISING
  );

  // Motor
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  Serial.println("PID DE POSICION");
  Serial.println("Ingrese posicion deseada en grados:");
}


// ======================================================
// LOOP
// ======================================================

void loop() {

  // --------------------------------------------
  // LEER REFERENCIA
  // --------------------------------------------

  if (Serial.available() > 0) {

    referencia = Serial.parseFloat();

    while (Serial.available() > 0) {
      Serial.read();
    }

    // Reiniciar integral al cambiar referencia
    integral = 0;

    Serial.print("Nueva referencia: ");
    Serial.print(referencia);
    Serial.println(" grados");
  }


  // --------------------------------------------
  // PID CADA 50 ms
  // --------------------------------------------

  unsigned long tiempoActual = millis();

  if (tiempoActual - tiempoAnterior >= Ts) {

    tiempoAnterior = tiempoActual;


    // ==========================================
    // 1. LEER ENCODER
    // ==========================================

    noInterrupts();

    long cuentas = contadorEncoder;

    interrupts();


    // ==========================================
    // 2. CALCULAR POSICION
    // ==========================================

    posicion =
      cuentas * 360.0 / CUENTAS_POR_REV;


    // ==========================================
    // 3. ERROR
    // ==========================================

    error =
      referencia - posicion;


    // ==========================================
    // 4. INTEGRAL
    // ==========================================

    integral =
      integral + error * dt;

    integral =
      constrain(integral, -100.0, 100.0);


    // ==========================================
    // 5. DERIVATIVA
    // ==========================================

    derivativo =
      (error - errorAnterior) / dt;


    // ==========================================
    // 6. PID
    // ==========================================

    control =
      Kp * error +
      Ki * integral +
      Kd * derivativo;


    control =
      constrain(control, -255.0, 255.0);


    // ==========================================
    // 7. CONTROL DEL MOTOR
    // ==========================================

    // Tolerancia para evitar oscilacion
    if (abs(error) < 1.0) {

      analogWrite(ENA, 0);

      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);

      control = 0;
    }

    else if (control > 0) {

      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);

      analogWrite(
        ENA,
        (int)control
      );
    }

    else {

      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);

      analogWrite(
        ENA,
        (int)(-control)
      );
    }


    // ==========================================
    // 8. GUARDAR ERROR
    // ==========================================

    errorAnterior = error;


    // ==========================================
    // 9. MOSTRAR DATOS
    // ==========================================

    Serial.print("Ref: ");
    Serial.print(referencia, 1);

    Serial.print(" | Pos: ");
    Serial.print(posicion, 1);

    Serial.print(" | Error: ");
    Serial.print(error, 1);

    Serial.print(" | PWM: ");
    Serial.println(control, 1);
  }
}


// ======================================================
// INTERRUPCION DEL ENCODER
// ======================================================

void contarEncoder() {

  int estadoB = digitalRead(ENC_B);

  if (estadoB == LOW) {

    contadorEncoder++;
  }

  else {

    contadorEncoder--;
  }
}