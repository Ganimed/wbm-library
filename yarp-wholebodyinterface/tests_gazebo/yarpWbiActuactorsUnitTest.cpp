/*
 * Copyright (C) 2014 Open Source Robotics Foundation
 * Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gtest/gtest.h>

#include "GazeboYarpServerFixture.hh"

#include "gazebo_yarp_test_config.h"

#include <yarp/os/Time.h>

#include <wbi/wbiUtil.h>

#include "../include/yarpWholeBodyInterface/yarpWholeBodyActuators.h"
#include "../include/yarpWholeBodyInterface/yarpWholeBodySensors.h"

class yarpWbiActuatorsUnitTest : public GazeboYarpServerFixture
{
};

/////////////////////////////////////////////////
/*
TEST_F(yarpWbiActuatorsUnitTest, basicGazeboYarpLoadingTest)
{
  Load("double_pendulum.world", true);


  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);
}
*/

/////////////////////////////////////////////////
TEST_F(yarpWbiActuatorsUnitTest, basicWbiLoadingTest)
{
  Load("double_pendulum.world", false);


  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);


  bool is_yarp_network_active = yarp::os::NetworkBase::checkNetwork(1.0);
  ASSERT_TRUE(is_yarp_network_active);


  yarpWbi::yarpWholeBodyActuators doublePendulumActuactors("test_actuactors");
  yarpWbi::yarpWholeBodySensors    doublePendulumSensors("test_sensors");

  yarp::os::Property wbiInterfaceProperties;
  wbiInterfaceProperties.fromConfigFile(std::string(YARP_CONF_PATH)+"/"+"wbi_double_pendulum.ini",true);

  doublePendulumActuactors.setYarpWbiProperties(wbiInterfaceProperties);
  doublePendulumSensors.setYarpWbiProperties(wbiInterfaceProperties);

  ASSERT_TRUE(doublePendulumActuactors.addActuator(wbi::wbiId("upper_joint")));
  ASSERT_TRUE(doublePendulumActuactors.addActuator(wbi::wbiId("lower_joint")));
  //ASSERT_FALSE(doublePendulumActuactors.addActuator(wbi::wbiId("third_joint")));

  ASSERT_TRUE(doublePendulumSensors.addSensor(wbi::SENSOR_ENCODER_POS,wbi::wbiId("upper_joint")));
  ASSERT_TRUE(doublePendulumSensors.addSensor(wbi::SENSOR_ENCODER_POS,wbi::wbiId("lower_joint")));
  //ASSERT_FALSE(doublePendulumSensors.addSensor(wbi::SENSOR_ENCODER_POS,wbi::wbiId("third_joint")));

  //std::cout << "doublePendulumActuactors.init()" << std::endl;
  ASSERT_TRUE(doublePendulumActuactors.init());
  //std::cout << "doublePendulumSensors.init()" << std::endl;
  ASSERT_TRUE(doublePendulumSensors.init());

  yarp::sig::Vector real_q(doublePendulumSensors.getSensorNumber(wbi::SENSOR_ENCODER_POS),-10);
  yarp::sig::Vector desired_q(doublePendulumActuactors.getActuatorList().size());
  yarp::sig::Vector ref_dq(doublePendulumActuactors.getActuatorList().size(),40.0*M_PI/180.0);


  ASSERT_TRUE(doublePendulumSensors.readSensors(wbi::SENSOR_ENCODER_POS,real_q.data(),0,true));

  std::cout << "Read position " << real_q[0] << " " << real_q[1] << std::endl;

  desired_q[0] = real_q[0] + M_PI/2;
  desired_q[1] = real_q[1] + M_PI/2;

  ASSERT_TRUE(doublePendulumActuactors.setControlMode(wbi::CTRL_MODE_POS));
  ASSERT_TRUE(doublePendulumActuactors.setControlParam(wbi::CTRL_PARAM_REF_VEL, ref_dq.data()));
  ASSERT_TRUE(doublePendulumActuactors.setControlReference(desired_q.data()));

  //Wait to reach the desired position
  yarp::os::Time::delay(5.0);

  ASSERT_TRUE(doublePendulumSensors.readSensors(wbi::SENSOR_ENCODER_POS,real_q.data(),0,true));

  double tol = 0.1;

  EXPECT_NEAR(desired_q[0],real_q[0],tol);
  EXPECT_NEAR(desired_q[1],real_q[1],tol);

  desired_q[0] = real_q[0] - M_PI;
  desired_q[1] = real_q[1] - M_PI;

  ASSERT_TRUE(doublePendulumActuactors.setControlMode(wbi::CTRL_MODE_POS));
  ASSERT_TRUE(doublePendulumActuactors.setControlReference(desired_q.data()));

  //Wait to reach the desired position
  yarp::os::Time::delay(5.0);

  ASSERT_TRUE(doublePendulumSensors.readSensors(wbi::SENSOR_ENCODER_POS,real_q.data()));

  EXPECT_NEAR(desired_q[0],real_q[0],tol);
  EXPECT_NEAR(desired_q[1],real_q[1],tol);

  ASSERT_TRUE(doublePendulumSensors.close());
  ASSERT_TRUE(doublePendulumActuactors.close());
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
