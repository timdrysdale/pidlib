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

TEST(PIDTest, limit) {

  PID controller = PID(1.0,0.0,0.0,0.02,0.0,-1,1);

  controller.setCommand(2);
  
  float y = controller.update(-2);
  EXPECT_FLOAT_EQ(1.0, y);

}




int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
