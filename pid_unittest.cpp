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

  PID controller = PID(1.0,0.0,0.0,0.02,0.0,-1,1);

  controller.setCommand(2);
  
  float y = controller.update(-2);
  EXPECT_FLOAT_EQ(1.0, y);

}

TEST(PIDTest, proportional) {

  PID controller = PID(1.0,0.0,0.0,0.02,0.0,-1,1);

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
  PID controller = PID(0.0,Ki,0.0,Ts,0.0,-1,1);

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


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
