#include <QTRSensors.h>
#include <stdint.h>

// =====================================================================
//  Seguidor de línea BLANCA - Arduino Nano + TB6612FNG + QTR-8A
//  - Sigue la línea blanca con PID
//  - Curvas: velocidad dinámica + reversa de la rueda interior
//  - Si ve un poco de blanco, busca la línea girando y la sigue
//  - Si los 6 sensores ven lo mismo (negro u otro color) por más de
//    LOST_TIMEOUT_MS -> PARO TOTAL (se reanuda con el botón)
// =====================================================================

#define LINEA_BLANCA 1

// --------- Pines QTR-8A (6 sensores) ---------
#define NUM_SENSORS 6
#define CENTRO ((NUM_SENSORS - 1) * 1000 / 2)   // 2500
const uint8_t sensorPins[NUM_SENSORS] = { A1, A2, A3, A4, A5, A6 };
#define IR_PIN 11

// --------- Pines TB6612FNG ---------
#define PWMA 3
#define AIN1 4
#define AIN2 5
#define PWMB 9
#define BIN1 8
#define BIN2 7
#define STBY 6

#define LED    13
#define BUTTON 12     // a GND, INPUT_PULLUP

#define PWM_ALTA_FRECUENCIA 1

QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS];

// --------- PID ---------
float Kp = 0.30f;
float Ki = 0.00f;
float Kd = 0.60f;

long integral  = 0;
int  lastError = 0;

// --------- Velocidades (ajústalas aquí) ---------
const int VEL_RECTA   = 100;   // |error| < 500
const int VEL_SUAVE   = 75;    // |error| < 1500
const int VEL_CERRADA = 55;    // curvas cerradas
const int MAX_PWM_SAFE = 255;
const int MIN_PWM_ATRAS = -100; // límite de reversa de la rueda interior en curva

// --------- Línea perdida / paro ---------
const uint16_t LINE_THRESHOLD = 100;        // 0..1000, "veo línea" si algún sensor lo supera
const unsigned long LOST_TIMEOUT_MS = 400;  // tiempo buscando antes de parar
const int TURN_PWM = 80;                    // giro en el lugar al buscar la línea
const uint16_t UNIFORME_RANGO = 120;        // si max-min de los 6 sensores es menor, "todo igual"

unsigned long lostSince = 0;   // 0 = la línea se está viendo
int lastSide = 1;              // signo de la última corrección (+1 / -1)

// --------- Monitor Serial ---------
#define DEBUG_SERIAL 1
unsigned long lastPrintTime = 0;
const unsigned int PRINT_INTERVAL = 100;

const unsigned long PAUSA_ENTRE_FASES_MS = 3000;

// --------- Motores ---------
static inline void driveMotor(uint8_t in1, uint8_t in2, uint8_t pwmPin, int speed) {
  speed = constrain(speed, -MAX_PWM_SAFE, MAX_PWM_SAFE);
  if (speed >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -speed);
  }
}

void setMotors(int leftSpeed, int rightSpeed) {
  driveMotor(AIN1, AIN2, PWMA, leftSpeed);
  driveMotor(BIN1, BIN2, PWMB, rightSpeed);
}

void brakeMotors() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH); analogWrite(PWMA, 255);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH); analogWrite(PWMB, 255);
}

void coastMotors() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); analogWrite(PWMA, 0);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); analogWrite(PWMB, 0);
}

void parpadear(uint8_t veces) {
  for (uint8_t i = 0; i < veces; i++) {
    digitalWrite(LED, HIGH); delay(250);
    digitalWrite(LED, LOW);  delay(250);
  }
}

void calibrarFase() {
  for (int i = 0; i < 120; i++) {
    if ((i / 20) % 2 == 0) setMotors(60, -60);
    else                   setMotors(-60, 60);
    qtr.calibrate();
    delay(10);
  }
  brakeMotors();
  delay(100);
  coastMotors();
}

void imprimirCalibracion() {
  Serial.print(F("    Min: "));
  for (uint8_t i = 0; i < NUM_SENSORS; i++) { Serial.print(qtr.calibrationOn.minimum[i]); Serial.print(' '); }
  Serial.println();
  Serial.print(F("    Max: "));
  for (uint8_t i = 0; i < NUM_SENSORS; i++) { Serial.print(qtr.calibrationOn.maximum[i]); Serial.print(' '); }
  Serial.println();
}

// --------- Paro total: freno, motores libres y espera el botón ---------
void paroTotal() {
  brakeMotors();
  delay(150);
  coastMotors();
  Serial.println(F("PARO: los 6 sensores ven lo mismo. Presiona el boton para reanudar."));

  // Espera botón (parpadeo lento del LED)
  while (digitalRead(BUTTON) == HIGH) {
    digitalWrite(LED, (millis() / 300) % 2);
  }
  while (digitalRead(BUTTON) == LOW) { }   // espera a que lo suelte
  digitalWrite(LED, HIGH);
  delay(800);                              // tiempo para soltar el robot en la pista

  integral = 0;
  lastError = 0;
  lostSince = 0;
}

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  coastMotors();
  digitalWrite(STBY, HIGH);

#if PWM_ALTA_FRECUENCIA
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;  // Timer1 -> D9
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;  // Timer2 -> D3
#endif

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  pinMode(BUTTON, INPUT_PULLUP);

  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, NUM_SENSORS);
  qtr.setEmitterPin(IR_PIN);

  Serial.println(F("Robot encendido."));
  parpadear(2);

  Serial.println(F("=== CALIBRACION ==="));
  delay(3000);

  digitalWrite(LED, HIGH);
  Serial.println(F("[1/2] Calibrando LINEA BLANCA... (LED encendido)"));
  calibrarFase();
  imprimirCalibracion();

  digitalWrite(LED, LOW);
  if (PAUSA_ENTRE_FASES_MS > 0) {
    Serial.println(F("Cambia a la LINEA NEGRA..."));
    delay(PAUSA_ENTRE_FASES_MS);
  }
  Serial.println(F("[2/2] Calibrando LINEA NEGRA... (LED apagado)"));
  calibrarFase();
  imprimirCalibracion();
  Serial.println(F("=== CALIBRACION FINALIZADA ==="));

  parpadear(3);
  Serial.println(F("Arrancando - modo LINEA BLANCA"));
  digitalWrite(LED, HIGH);
}

void loop() {
#if LINEA_BLANCA
  uint16_t position = qtr.readLineWhite(sensorValues);
#else
  uint16_t position = qtr.readLineBlack(sensorValues);
#endif

  // ¿Algún sensor ve la línea?
  uint16_t lineMax = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
#if LINEA_BLANCA
    uint16_t v = 1000 - sensorValues[i];
#else
    uint16_t v = sensorValues[i];
#endif
    if (v > lineMax) lineMax = v;
  }

  // ¿Los 6 sensores leen casi lo mismo? (todo negro u otro color uniforme)
  uint16_t vMin = 1000, vMax = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] < vMin) vMin = sensorValues[i];
    if (sensorValues[i] > vMax) vMax = sensorValues[i];
  }
  bool todoIgual = (vMax - vMin) < UNIFORME_RANGO;

  // --- Sin línea o todo igual: busca un instante; si no aparece, PARO TOTAL ---
  if (lineMax < LINE_THRESHOLD || todoIgual) {
    unsigned long ahora = millis();
    if (lostSince == 0) lostSince = ahora;

    if (ahora - lostSince >= LOST_TIMEOUT_MS) {
      paroTotal();
      return;
    }
    // Giro en el lugar hacia donde se fue la línea (mismo sentido que el PID)
    if (lastSide > 0) setMotors(-TURN_PWM,  TURN_PWM);
    else              setMotors( TURN_PWM, -TURN_PWM);
    return;
  }
  lostSince = 0;

  // --- PID ---
  int error = (int)position - CENTRO;   // -2500..2500
  integral += error;
  integral = constrain(integral, -20000L, 20000L);
  int derivative = error - lastError;
  int correction = (int)(Kp * error + Ki * integral + Kd * derivative);
  lastError = error;

  if (correction > 0)      lastSide =  1;
  else if (correction < 0) lastSide = -1;

  // --- Velocidad dinámica ---
  int baseSpeed;
  if (abs(error) < 500)       baseSpeed = VEL_RECTA;
  else if (abs(error) < 1500) baseSpeed = VEL_SUAVE;
  else                        baseSpeed = VEL_CERRADA;

  // La rueda interior puede ir en reversa (hasta MIN_PWM_ATRAS) para cerrar curvas
  int left  = constrain(baseSpeed - correction, MIN_PWM_ATRAS, MAX_PWM_SAFE);
  int right = constrain(baseSpeed + correction, MIN_PWM_ATRAS, MAX_PWM_SAFE);

#if DEBUG_SERIAL
  if (millis() - lastPrintTime > PRINT_INTERVAL) {
    Serial.print(F("Pos: "));   Serial.print(position);
    Serial.print(F("  Err: ")); Serial.print(error);
    Serial.print(F("  Corr: "));Serial.print(correction);
    Serial.print(F("  Mot: ["));Serial.print(left);
    Serial.print(F(", "));      Serial.print(right);
    Serial.println(F("]"));
    lastPrintTime = millis();
  }
#endif

  setMotors(left, right);
}
