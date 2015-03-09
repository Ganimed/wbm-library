#include "wbiConstants.h"

namespace wbi {

    const SensorType SENSOR_ENCODER = SENSOR_ENCODER_POS;

    const SensorTypeDescription sensorTypeDescriptions[SENSOR_TYPE_SIZE] =
    {
        SensorTypeDescription(SENSOR_ENCODER_POS,   "position encoder", 1, true,  "Joint position"),
        SensorTypeDescription(SENSOR_ENCODER_SPEED, "speed encoder",    1, true,  "Joint velocity"),
        SensorTypeDescription(SENSOR_ENCODER_ACCELERATION, "acceleration encoder", 1, true,  "Joint acceleration"),
        SensorTypeDescription(SENSOR_PWM,           "PWM",              1, true,  "Motor PWM"),
        SensorTypeDescription(SENSOR_CURRENT,       "current",          1, true,  "Motor current"),
        SensorTypeDescription(SENSOR_TORQUE,        "torque",           1, true,  "Joint torque"),
        SensorTypeDescription(SENSOR_IMU,           "IMU",              13, false, "Inertial Measurement Unit"),
        SensorTypeDescription(SENSOR_FORCE_TORQUE,  "force-torque",     6, false, "6-axis force torque"),
        SensorTypeDescription(SENSOR_ACCELEROMETER, "accelerometer",    3, false, "3d linear acceleration"),
    };


    SensorTypeDescription::SensorTypeDescription(SensorType _id, std::string _name, int _dataSize, bool _isJoint, std::string _descr)
    : id(_id)
    , name(_name)
    , description(_descr)
    , dataSize(_dataSize)
    , isJointSensor(_isJoint)
    {}

    bool SensorTypeDescription::operator ==(const SensorTypeDescription &st)
    {
        return st.id == this->id;
    }

}