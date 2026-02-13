#include <Servo.h>

// ==== Pinos (seu esquema) ====
#define PIN_TRIG   9
#define PIN_ECHO   10
#define PIN_SERVO  8
#define PIN_BUZZER 7

// ==== Distância ====
const float MIN_CM = 5.0;
const float MAX_CM = 200.0;

// ==== Servo ====
Servo myServo;
const int SERVO_MIN_DEG = 0;
const int SERVO_MAX_DEG = 170;

// ---- Movimento “de braço” ----
const float SERVO_MAX_SPEED_DPS   = 240.0;   // °/s
const float SERVO_MAX_ACCEL_DPS2  = 800.0;   // °/s²
const int   SERVO_DEADBAND_DEG    = 1;
const uint16_t CONTROL_PERIOD_MS  = 15;      // ~66 Hz

float currentAngle = 90.0;   // estado simulado
float currentSpeed = 0.0;    // °/s
unsigned long lastControlTick = 0;

// ==== Buzzer ====
const int TONE_NEAR_HZ = 1200;
const int TONE_BEEP_HZ = 2200;
unsigned long lastBeepToggle = 0;
bool beepingOn = false;

// ==== Filtro de distância ====
float distFiltered = 60.0;
const float ALPHA = 0.25f; // 0–1

// ==== Estados de comportamento ====
enum ArmState { IDLE_HOME, ENGAGED_TRACKING, RETURNING_HOME };
ArmState armState = IDLE_HOME;

// Parâmetros do “voltar à posição”
const float HOME_ANGLE = 30.0;        // posição “recolhida” do braço
const float ENGAGE_CM = 40.0;         // entra em rastreamento se ficar <= 40 cm
const float DISENGAGE_CM = 55.0;      // começa a considerar retorno se >= 55 cm
const unsigned long RETURN_DELAY_MS = 800; // deve ficar longe por esse tempo

unsigned long farSince = 0; // quando começou a ficar longe

// -------- Funções de distância --------
float readDistanceCm()
{
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long duration = pulseIn(PIN_ECHO, HIGH, 30000UL); // timeout ~30 ms
  if (duration == 0) return NAN;

  float distanceCm = duration / 58.0; // HC-SR04 típico
  if (distanceCm < MIN_CM || distanceCm > 400.0) return NAN;
  return distanceCm;
}

// Média rápida + filtro exponencial
bool sampleAndFilterDistance(float &outCm)
{
  const uint8_t SAMPLES = 4;
  float sum = 0; uint8_t ok = 0;
  for (uint8_t i = 0; i < SAMPLES; i++) {
    float d = readDistanceCm();
    if (!isnan(d)) { sum += d; ok++; }
    delay(5);
  }
  if (ok == 0) return false;

  float avg = sum / ok;
  if (avg < MIN_CM) avg = MIN_CM;
  if (avg > MAX_CM) avg = MAX_CM;

  distFiltered = ALPHA * avg + (1.0f - ALPHA) * distFiltered;
  outCm = distFiltered;
  return true;
}

// -------- Utilidades --------
long fmap(float x, float in_min, float in_max, long out_min, long out_max)
{
  if (x < in_min) x = in_min;
  if (x > in_max) x = in_max;
  return (long)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

// Perfil trapezoidal (limite vel + acel)
void servoMotionControllerStep(float targetAngle)
{
  unsigned long now = millis();
  if (now - lastControlTick < CONTROL_PERIOD_MS) return;
  float dt = (now - lastControlTick) / 1000.0f;
  lastControlTick = now;

  if (targetAngle < SERVO_MIN_DEG) targetAngle = SERVO_MIN_DEG;
  if (targetAngle > SERVO_MAX_DEG) targetAngle = SERVO_MAX_DEG;

  float posError = targetAngle - currentAngle;

  if (fabs(posError) <= SERVO_DEADBAND_DEG) {
    currentSpeed = 0.0f;
    myServo.write((int)round(targetAngle));
    currentAngle = targetAngle;
    return;
  }

  float dir = (posError > 0) ? 1.0f : -1.0f;

  float maxDeltaV = SERVO_MAX_ACCEL_DPS2 * dt;
  float desiredSpeed = currentSpeed + dir * maxDeltaV;

  if (desiredSpeed >  SERVO_MAX_SPEED_DPS) desiredSpeed =  SERVO_MAX_SPEED_DPS;
  if (desiredSpeed < -SERVO_MAX_SPEED_DPS) desiredSpeed = -SERVO_MAX_SPEED_DPS;

  // Frenagem antecipada
  float vStop = sqrtf(2.0f * SERVO_MAX_ACCEL_DPS2 * fabs(posError));
  if (fabs(desiredSpeed) > vStop) desiredSpeed = vStop * dir;

  currentSpeed = desiredSpeed;
  currentAngle += currentSpeed * dt;

  if (currentAngle < SERVO_MIN_DEG) currentAngle = SERVO_MIN_DEG;
  if (currentAngle > SERVO_MAX_DEG) currentAngle = SERVO_MAX_DEG;

  myServo.write((int)round(currentAngle));
}

// -------- Buzzer --------
void buzzerByDistance(float dist)
{
  if (dist <= 10.0) {
    tone(PIN_BUZZER, TONE_NEAR_HZ);
  } else if (dist <= 50.0) {
    int intervalMs = (int)fmap(dist, 10.0, 50.0, 60, 400);
    unsigned long now = millis();
    if (now - lastBeepToggle >= (unsigned long)intervalMs) {
      lastBeepToggle = now;
      beepingOn = !beepingOn;
      if (beepingOn) tone(PIN_BUZZER, TONE_BEEP_HZ);
      else noTone(PIN_BUZZER);
    }
  } else {
    noTone(PIN_BUZZER);
  }
}

// -------- Lógica de estados do braço --------
float computeTargetAngleForState(float distCm)
{
  switch (armState) {
    case IDLE_HOME:
      // Fica “em casa” até alguém chegar perto (≤ ENGAGE_CM)
      if (distCm <= ENGAGE_CM) {
        armState = ENGAGED_TRACKING;
        farSince = 0;
      }
      return HOME_ANGLE;

    case ENGAGED_TRACKING:
      // Rastreia (mapa distância->ângulo)
      if (distCm >= DISENGAGE_CM) {
        // começou a ficar longe
        if (farSince == 0) farSince = millis();
        // se ficou longe por tempo suficiente, volta pra casa
        if (millis() - farSince >= RETURN_DELAY_MS) {
          armState = RETURNING_HOME;
        }
      } else {
        // voltou a ficar perto: zera contador
        farSince = 0;
      }
      // Longe (200 cm) -> 0°, Perto (5 cm) -> 170°
      return (float)fmap(distCm, MAX_CM, MIN_CM, SERVO_MIN_DEG, SERVO_MAX_DEG);

    case RETURNING_HOME:
      // Vai pra casa; se alguém se aproximar novamente, volta a rastrear
      if (distCm <= ENGAGE_CM) {
        armState = ENGAGED_TRACKING;
        farSince = 0;
        // nesse instante o alvo vira o mapeado (quem manda é o return abaixo do case TRACKING)
        return (float)fmap(distCm, MAX_CM, MIN_CM, SERVO_MIN_DEG, SERVO_MAX_DEG);
      }
      // Se já chegou em casa, fica em casa (IDLE_HOME)
      if (fabs(currentAngle - HOME_ANGLE) <= SERVO_DEADBAND_DEG) {
        armState = IDLE_HOME;
      }
      return HOME_ANGLE;
  }
  // fallback
  return HOME_ANGLE;
}

void setup() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  myServo.attach(PIN_SERVO);
  myServo.write((int)currentAngle);

  Serial.begin(9600);
  Serial.println("Robo: estados (HOME/ENGAGED/RETURNING) + movimento suave + buzzer");
  lastControlTick = millis();
}

void loop() {
  float distCm;
  bool ok = sampleAndFilterDistance(distCm);

  if (!ok) {
    // Sem leitura confiável: permanece na lógica atual, mas sem buzzer estridente
    noTone(PIN_BUZZER);
    // Se perder leitura por muito tempo, retornamos para HOME
    static unsigned long lostSince = 0;
    if (lostSince == 0) lostSince = millis();
    if (millis() - lostSince > 800) {
      armState = RETURNING_HOME;
    }
    // Continua controlando o servo rumo ao alvo do estado atual
    float target = computeTargetAngleForState(distFiltered);
    servoMotionControllerStep(target);
    delay(5);
    return;
  } else {
    // reset se voltou a ler
    static unsigned long lostSince = 0;
    lostSince = 0;
  }

  // Buzzer reativo (mesma lógica)
  buzzerByDistance(distCm);

  // Alvo do servo conforme estado
  float targetAngle = computeTargetAngleForState(distCm);

  // Passo do controlador
  servoMotionControllerStep(targetAngle);

  // Debug leve
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    Serial.print("State: ");
    Serial.print(armState == IDLE_HOME ? "HOME" :
                 armState == ENGAGED_TRACKING ? "TRACK" : "RETURN");
    Serial.print(" | Dist: ");
    Serial.print(distCm, 1);
    Serial.print(" | Target: ");
    Serial.print(targetAngle, 1);
    Serial.print(" | Angle: ");
    Serial.print(currentAngle, 1);
    Serial.print(" | Speed: ");
    Serial.println(currentSpeed, 1);
  }

  delay(3);
}
