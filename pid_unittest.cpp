//pid_unittest.cpp

#include "pid.cpp"
#include <gtest/gtest.h>

TEST(PIDTest, atSetPoint) {

  PID controller = PID(1.0,0.0,0.0,0.02,20.0,-0.5,0.5);

  controller.setCommand(0.5);
  
  float y = controller.update(0.5);
  EXPECT_FLOAT_EQ(0.0, y);

  for (int i = 1; i < 11; ++i)
	{
	  float y = controller.update(0.5);
	  EXPECT_FLOAT_EQ(0.0, y );
  }
}


TEST(PIDTest, limit) {
  float Ts = 0.02;
  float N = 20;
  PID controller = PID(1.0,0.0,0.0,Ts,N,-1,1);

  controller.setCommand(2);
  
  float y = controller.update(-2);
  EXPECT_FLOAT_EQ(1.0, y);
}


TEST(PIDTest, proportional) {
  float Ts = 0.02;
  float N = 20; 
  PID controller = PID(1.0,0.0,0.0,Ts,N,-1,1);

  float setpoint = 0.5;
  
  controller.setCommand(setpoint);
  
  float y = controller.update(setpoint);
  
  EXPECT_FLOAT_EQ(0.0, y);


  for (int i = 1; i < 11; ++i)
	{
	  float plant = float(0.01 * i);
	  float y = controller.update(plant);
	  float exp = setpoint - plant;
	  EXPECT_FLOAT_EQ(exp, y );
  }
}


TEST(PIDTest, integral) {

  float Ts = 0.02;
  float Ki = 10.0;
  float N = 0; //with N >0 there is a small difference which would require a tolerance on the equality test
  PID controller = PID(0.0,Ki,0.0,Ts,N,-1,1);

  float setpoint = 0.0;
  
  controller.setCommand(setpoint);
  
  float y = controller.update(setpoint);
  
  EXPECT_FLOAT_EQ(0.0, y);

  float plant = 0.1;
  float error = plant - setpoint;
	
  for (int i = 1; i < 51; ++i)
  {
	  float y = controller.update(plant);
	  float exp = -1.0 * i * error * Ts * Ki;
	  EXPECT_FLOAT_EQ(exp, y);
  }

  // now at limit
  
  float exp = -1.0 * 50 * error * Ts * Ki;
  for (int i = 1; i < 51; ++i)
  {
	  float y = controller.update(plant);
	  EXPECT_FLOAT_EQ(exp, y);
  }

  y = controller.update(setpoint);
  // grace of one step for this implementation
  // due to error history

  // expect no error because have reached setpoint
  // so integrator has been reset
  y = controller.update(setpoint);
  EXPECT_FLOAT_EQ(0, y);

  // repeat test but check for reset on zero crossing
  plant = 0.1;
  error = plant - setpoint;

  for (int i = 1; i < 51; ++i)
  {
	  float y = controller.update(plant);
	  float exp = -1.0 * i * error * Ts * Ki;
	  EXPECT_FLOAT_EQ(exp, y);
  }

  // now at limit
  
  exp = -1.0 * 50 * error * Ts * Ki;
  for (int i = 1; i < 51; ++i)
  {
	  float y = controller.update(plant);
	  EXPECT_FLOAT_EQ(exp, y);
  }

  y = controller.update(-0.1);
  float oneIntegralStep = 1.0 * error * Ts * Ki; //with no history, you'd get this
  // need a positive control output now,
  // due to history it won't be exactly one integral step
  // let it be up to two of them ...
  EXPECT_TRUE((y > 0) && (y <= 2 * oneIntegralStep));
}


TEST(PIDTest, derivative) {

  float Ts = 0.02;
  float Kd = 1.0;
  float N = 20;
  PID controller = PID(0.0,0.0,Kd,Ts,N,-1,1);

  float setpoint = 0.0;
  
  controller.setCommand(setpoint);
  
  float y = controller.update(setpoint);

  EXPECT_FLOAT_EQ(0.0, y);
  
  float deltaPlant = 0.01;
  float plant = 0;
  float deltaY = 0;
  float lastDeltaY = 0;
  float lastY = 0;

  // low pass filter has a transient response
  // check that it is monotonic and asymptotic
  // then check steady state derivative is as expected
  for (int i = 1; i < 91; ++i)
  {
	plant += deltaPlant;
	lastDeltaY = deltaY;
	lastY = y;
	y = controller.update(plant);
	lastDeltaY = deltaY;
	deltaY = y - lastY;

	if (deltaY > 1e-6 && i > 2) {
	  EXPECT_TRUE(y < lastY) << i << ":" << plant << ":" << y << ":" << lastY; //monotonic
	  EXPECT_TRUE(deltaY >= lastDeltaY) << i << ":"<< deltaY << ":" << lastDeltaY; //asymptotic
	}
  }

  // Should be at the steady state now, so 
  // check that output is the expected derivative
  float expected = -1 * deltaPlant / Ts;
  EXPECT_TRUE(abs(expected-y)<1e-4);
}


TEST(PIDTest, setget) {

  float Kp = 1.0;
  float Ki = 2.0;
  float Kd = 3.0; 
  float Ts = 0.02;
  float N = 20;
  float uMin = -1;
  float uMax = 1;
  
  PID controller = PID(Kp,Ki,Kd,Ts,N,uMin,uMax);

  EXPECT_FLOAT_EQ(Kp, controller.getKp());
  EXPECT_FLOAT_EQ(Ki, controller.getKi());
  EXPECT_FLOAT_EQ(Kd, controller.getKd());
  EXPECT_FLOAT_EQ(Ts, controller.getTs());
  EXPECT_FLOAT_EQ(N, controller.getN());
  EXPECT_FLOAT_EQ(uMin, controller.getUMin());
  EXPECT_FLOAT_EQ(uMax, controller.getUMax());

  EXPECT_TRUE(controller.hasZeroHistory());
  controller.setCommand(1.0);
  controller.update(-1.0);
  EXPECT_FALSE(controller.hasZeroHistory());

  Kp = 4.0;
  Ki = 5.0;
  Kd = 6.0;
  
  controller.setKs(Kp,Ki,Kd);
  EXPECT_FLOAT_EQ(Kp, controller.getKp());
  EXPECT_FLOAT_EQ(Ki, controller.getKi());
  EXPECT_FLOAT_EQ(Kd, controller.getKd());
  
  EXPECT_TRUE(controller.hasZeroHistory());
  controller.setCommand(1.0);
  controller.update(-1.0);
  EXPECT_FALSE(controller.hasZeroHistory());

  Kp = 7.0;
  controller.setKp(Kp);
  EXPECT_FLOAT_EQ(Kp, controller.getKp());
  EXPECT_TRUE(controller.hasZeroHistory());
  controller.setCommand(1.0);
  controller.update(-1.0);
  EXPECT_FALSE(controller.hasZeroHistory());  
  
  Ki = 8.0;
  controller.setKi(Ki);
  EXPECT_FLOAT_EQ(Ki, controller.getKi());
  EXPECT_TRUE(controller.hasZeroHistory());
  controller.setCommand(1.0);
  controller.update(-1.0);
  EXPECT_FALSE(controller.hasZeroHistory());

  Kd = 9.0;
  controller.setKd(Kd);
  EXPECT_FLOAT_EQ(Kd, controller.getKd());
  EXPECT_TRUE(controller.hasZeroHistory());
  controller.setCommand(1.0);
  controller.update(-1.0);
  EXPECT_FALSE(controller.hasZeroHistory());

  Ts = 0.001;
  controller.setTs(Ts);
  EXPECT_FLOAT_EQ(Ts, controller.getTs());
  EXPECT_TRUE(controller.hasZeroHistory());
  controller.setCommand(1.0);
  controller.update(-1.0);
  EXPECT_FALSE(controller.hasZeroHistory());

  N = 30;
  controller.setN(N);
  EXPECT_FLOAT_EQ(N, controller.getN());
  EXPECT_TRUE(controller.hasZeroHistory());
  controller.setCommand(1.0);
  controller.update(-1.0);
  EXPECT_FALSE(controller.hasZeroHistory());

  uMin = -2;
  uMax = 3;
  controller.setLimits(uMin,uMax);
  EXPECT_FLOAT_EQ(uMin, controller.getUMin());
  EXPECT_FLOAT_EQ(uMax, controller.getUMax());  
  EXPECT_TRUE(controller.hasZeroHistory());
  controller.setCommand(1.0);
  controller.update(-1.0);
  EXPECT_FALSE(controller.hasZeroHistory());  
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
