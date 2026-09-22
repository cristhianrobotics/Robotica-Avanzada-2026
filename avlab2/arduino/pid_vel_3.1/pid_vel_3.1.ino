// ======================================================
// ACTIVIDAD 3.1
// CONTROL PID DE VELOCIDAD
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
// Valor experimental obtenido en Actividad 2.1
const float CUENTAS_POR_REV = 925.0;


// ---------------- TIEMPO DE MUESTREO ----------------
// 100 ms = 10 Hz
const float dt = 0.1;

unsigned long tiempoAnterior = 0;


// ---------------- CONTADOR ENCODER ----------------
volatile long contadorEncoder = 0;

long contadorAnterior = 0;


// ---------------- VELOCIDAD ----------------
float velocidad = 0.0;


// ---------------- REFERENCIA ----------------
float referencia = 0.0;


// ---------------- PID ----------------

// Valores iniciales
float Kp = 30.0;
float Ki = 8.0;
float Kd = 0.0;

float error = 0.0;
float errorAnterior = 0.0;

float integral = 0.0;
float derivativo = 0.0;

float control = 0.0;


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


  // Motor inicialmente detenido
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);


  Serial.println("CONTROL PID DE VELOCIDAD");
  Serial.println("Ingrese referencia en rad/s:");
}


// ======================================================
// LOOP
// ======================================================

void loop() {


  // --------------------------------------------------
  // LEER REFERENCIA DESDE MONITOR SERIAL
  // --------------------------------------------------

  if (Serial.available() > 0) {

    referencia = Serial.parseFloat();

    while (Serial.available() > 0) {
      Serial.read();
    }

    Serial.print("Nueva referencia: ");
    Serial.print(referencia);
    Serial.println(" rad/s");
  }


  // --------------------------------------------------
  // EJECUTAR PID CADA 100 ms
  // --------------------------------------------------

  unsigned long tiempoActual = millis();


  if (tiempoActual - tiempoAnterior >= 100) {

    tiempoAnterior = tiempoActual;


    // ================================================
    // 1. LEER ENCODER
    // ================================================

    noInterrupts();

    long contadorActual = contadorEncoder;

    interrupts();


    long deltaCuentas =
      contadorActual - contadorAnterior;


    contadorAnterior = contadorActual;


    // ================================================
    // 2. CALCULAR VELOCIDAD
    // ================================================

    float cuentasSegundo =
      deltaCuentas / dt;


    velocidad =
      cuentasSegundo *
      (2.0 * PI) /
      CUENTAS_POR_REV;


    // ================================================
    // 3. CALCULAR ERROR
    // ================================================

    error =
      referencia - velocidad;


    // ================================================
    // 4. PARTE INTEGRAL
    // ================================================

    integral =
      integral + error * dt;


    // Limitación anti-windup
    integral =
      constrain(integral, -30.0, 30.0);


    // ================================================
    // 5. PARTE DERIVATIVA
    // ================================================

    derivativo =
      (error - errorAnterior) / dt;


    // ================================================
    // 6. PID
    // ================================================

    control =
      Kp * error +
      Ki * integral +
      Kd * derivativo;


    // Limitar a rango PWM
    control =
      constrain(control, -255.0, 255.0);


    // ================================================
    // 7. CONTROL MOTOR
    // ================================================

    if (control > 0) {

      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);

      analogWrite(
        ENA,
        (int)control
      );
    }

    else if (control < 0) {

      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);

      analogWrite(
        ENA,
        (int)(-control)
      );
    }

    else {

      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);

      analogWrite(ENA, 0);
    }


    // ================================================
    // 8. GUARDAR ERROR
    // ================================================

    errorAnterior = error;


    // ================================================
    // 9. MOSTRAR DATOS
    // ================================================

    Serial.print("Ref: ");
    Serial.print(referencia, 2);

    Serial.print(" | Vel: ");
    Serial.print(velocidad, 2);

    Serial.print(" | Error: ");
    Serial.print(error, 2);

    Serial.print(" | PWM: ");
    Serial.println(control, 1);
  }
}


// ======================================================
// INTERRUPCION DEL ENCODER
// ======================================================

void contarEncoder() {

  int estadoB =
    digitalRead(ENC_B);


  if (estadoB == LOW) {

    contadorEncoder++;
  }

  else {

    contadorEncoder--;
  }
}