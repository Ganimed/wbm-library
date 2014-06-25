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
TEST_F(yarpWbiActuatorsUnitTest, basicLoadingTest)
{
  Load("double_pendulum.world", true);

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

  ASSERT_TRUE(doublePendulumActuactors.addActuator(wbi::wbiId("first_joint")));
  ASSERT_TRUE(doublePendulumActuactors.addActuator(wbi::wbiId("second_joint")));
  //ASSERT_FALSE(doublePendulumActuactors.addActuator(wbi::wbiId("third_joint")));

  ASSERT_TRUE(doublePendulumSensors.addSensor(wbi::SENSOR_ENCODER,wbi::wbiId("first_joint")));
  ASSERT_TRUE(doublePendulumSensors.addSensor(wbi::SENSOR_ENCODER,wbi::wbiId("second_joint")));
  //ASSERT_FALSE(doublePendulumSensors.addSensor(wbi::SENSOR_ENCODER,wbi::wbiId("third_joint")));

  //std::cout << "doublePendulumActuactors.init()" << std::endl;
  ASSERT_TRUE(doublePendulumActuactors.init());
  //std::cout << "doublePendulumSensors.init()" << std::endl;
  ASSERT_TRUE(doublePendulumSensors.init());

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
