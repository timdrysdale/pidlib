#include "pid.h"


//PID::PID(float Kp, float Ki, float Kd, float Ts) {
//  return PID(Kp, Ki, Kd, Ts, 20);
//}

PID::PID(float Kp, float Ki, float Kd, float Ts, float N, float plantMin, float plantMax) {
  setAll(Kp, Ki, Kd, Ts, N, plantMin, plantMax);
}

void PID::setAll(float Kp, float Ki, float Kd, float Ts, float N, float plantMin, float plantMax) {

  reset();

  uMin = plantMin;
  uMax = plantMax;

  a0 = (1+N*Ts);
  a1 = -(2 + N*Ts);
  a2 = 1;
  b0 = Kp*(1+N*Ts) + Ki*Ts*(1+N*Ts) + Kd*N;
  b1 = -(Kp*(2+N*Ts) + Ki*Ts + 2*Kd*N);
  b2 = Kp + Kd*N;
  ku1 = a1/a0;
  ku2 = a2/a0;
  ke0 = b0/a0;
  ke1 = b1/a0;
  ke2 = b2/a0;

}

void PID::reset(void) { //TODO consider making this private
  e0 = 0;
  e1 = 0;
  e2 = 0; 
  u0 = 0;
  u1 = 0;
  u2 = 0;
}

void PID::setCommand(float command) {
  r = command;
}

float PID::update(float y){

  // shift history
  e2=e1;
  e1=e0;
  u2=u1;
  u1=u0;

  e0=r-y; //compute new error
  u0 = -ku1*u1 - ku2*u2 + ke0*e0 + ke1*e1 + ke2*e2; //eq(12)

  if (u0 > uMax) u0 = uMax; //limit to plant range
  if (u0 < uMin) u0 = uMin;

  return u0;

}
