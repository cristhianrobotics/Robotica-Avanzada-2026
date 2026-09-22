// ======================================================
// ACTIVIDAD 2.2 - LECTURA DE ENCODER Y VELOCIDAD ANGULAR
// Arduino UNO + L298N + Motor DC con Encoder
// ======================================================

// ---------------- MOTOR L298N ----------------
#define IN1 7
#define IN2 8
#define ENA 9

// ---------------- ENCODER ----------------
// Según la actividad 2.2:
#define ENC_A 3
#define ENC_B 2

// ---------------- CONFIGURACION ----------------

// Valor aproximado obtenido experimentalmente en 2.1
const float CUENTAS_POR_REV = 925.0;

// Frecuencia de cálculo: 10 Hz
const float FRECUENCIA = 10.0;

// PWM aplicado al motor
int pwmMotor = 255;

// ---------------- VARIABLES ----------------

volatile long contadorEncoder = 0;

long contadorAnterior = 0;

unsigned long tiempoAnterior = 0;


// ======================================================
// SETUP
// ======================================================

void setup() {

  Serial.begin(9600);

  // Encoder
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT);

  // Motor
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Interrupción del encoder
  attachInterrupt(
    digitalPinToInterrupt(ENC_A),
    contarEncoder,
    RISING
  );

  // Motor gira en un sentido
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  analogWrite(ENA, pwmMotor);

  Serial.println("Iniciando lectura del encoder...");
}


// ======================================================
// LOOP
// ======================================================

void loop() {

  unsigned long tiempoActual = millis();

  // 10 Hz -> cada 100 ms
  if (tiempoActual - tiempoAnterior >= 100) {

    tiempoAnterior = tiempoActual;

    // Copiar contador de manera segura
    noInterrupts();
    long contadorActual = contadorEncoder;
    interrupts();

    // Incremento de cuentas en estos 100 ms
    long deltaCuentas = contadorActual - contadorAnterior;

    contadorAnterior = contadorActual;

    // Tiempo de muestreo
    float deltaTiempo = 1.0 / FRECUENCIA;

    // Cuentas por segundo
    float cuentasSegundo = deltaCuentas / deltaTiempo;

    // Velocidad angular en rad/s
    float velocidadAngular =
      cuentasSegundo * (2.0 * PI) / CUENTAS_POR_REV;


    // Mostrar resultados
    Serial.print("Cuentas: ");
    Serial.print(contadorActual);

    Serial.print("   Delta: ");
    Serial.print(deltaCuentas);

    Serial.print("   Velocidad: ");
    Serial.print(velocidadAngular, 3);

    Serial.println(" rad/s");
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