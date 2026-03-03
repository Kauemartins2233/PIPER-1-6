#include "PID.h"

PID::PID(float Kp_, float Ki_, float Kd_, float Kv_, float outMin_, float outMax_, char id_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  Kv = Kv_;
  outMin = outMin_;
  outMax = outMax_;

  iMin = -100.0f;
  iMax = 100.0f;

  integrator = 0.0f;
  prevError = 0.0f;
  id = id_;
}

void PID::setIntegratorLimits(float minI, float maxI) {
  iMin = minI;
  iMax = maxI;
}

void PID::setTunings(float Kp_, float Ki_, float Kd_, float Kv_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  Kv = Kv_;
}

void PID::setOutputLimits(float minOut, float maxOut) {
  outMin = minOut;
  outMax = maxOut;
}

void PID::reset() {
  integrator = 0.0f;
  prevError = 0.0f;
}

float PID::update(float setpoint, float measurement, float vel, float dt) {
  float error = setpoint - measurement;

  // Integral "pura"
  float integratorCandidate = integrator + error * dt;

  // Derivada (se quiser ativar depois)
  float derivative = (error - prevError) / dt;

  // Aplica anti-windup com limite no integrador
  // Primeiro calcula saída não saturada com a integral candidata
  float u_unsat = Kp * error + Ki * integratorCandidate + Kd * derivative - Kv*vel;

  // Saída saturada
  float u = u_unsat;
  if (u > outMax) u = outMax;
  if (u < outMin) u = outMin;

  bool saturadoMax = (u >= outMax);
  bool saturadoMin = (u <= outMin);

  // Se estiver saturado e o erro empurra MAIS na direção da saturação, não integra
  if ((saturadoMax && error > 0) || (saturadoMin && error < 0)) {
    // não atualiza integrador
  } else {
    // Atualiza e CLAMPA o integrador
    integrator = integratorCandidate;
    if (integrator > iMax) integrator = iMax;
    if (integrator < iMin) integrator = iMin;
  }

  prevError = error;

#ifdef PID_DEBUG
    Serial.print(F("PID S: "));
    Serial.print(setpoint);
    Serial.print(F(" m: "));
    Serial.print(measurement);
    Serial.print(F(" dt: "));
    Serial.print(dt);
    Serial.print(F(" err: "));
    Serial.print(error);
    Serial.print(F(" I: "));
    Serial.print(integrator);
    Serial.print(F(" u_usat: "));
    Serial.print(u_unsat);
    Serial.print(F(" u: "));
    Serial.print(u);
    Serial.print(F(" id: "));
    Serial.println(id);

#endif

  return u;
}