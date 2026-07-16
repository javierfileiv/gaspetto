#include "mock_IMUOrientation.h"

MockIMUOrientation::MockIMUOrientation()
        : MockBase<MockIMUOrientation>()
{
    set_instance(this);
}

MockIMUOrientation::~MockIMUOrientation()
{
    clear_instance(this);
}

bool MockIMUOrientation::begin(uint8_t addr, TwoWire *theWire)
{
    auto mock = MockIMUOrientation::get_instance();
    return mock->_begin(addr, theWire);
}

void MockIMUOrientation::calibrate(bool print)
{
    auto mock = MockIMUOrientation::get_instance();
    mock->_calibrate(print);
}

void MockIMUOrientation::update()
{
    auto mock = MockIMUOrientation::get_instance();
    mock->_update();
}

float MockIMUOrientation::roll() const
{
    auto mock = MockIMUOrientation::get_instance();
    return mock->_roll();
}

float MockIMUOrientation::pitch() const
{
    auto mock = MockIMUOrientation::get_instance();
    return mock->_pitch();
}

float MockIMUOrientation::yaw() const
{
    auto mock = MockIMUOrientation::get_instance();
    return mock->_yaw();
}

float MockIMUOrientation::gyroZDeg() const
{
    auto mock = MockIMUOrientation::get_instance();
    return mock->_gyroZDeg();
}

void MockIMUOrientation::zeroYaw()
{
    auto mock = MockIMUOrientation::get_instance();
    mock->_zeroYaw();
}
