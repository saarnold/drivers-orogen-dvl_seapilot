/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace dvl_seapilot;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    std::cout << _device.get() << std::endl;

    driver.open(_device.get());

    driver.startMeasurement();

    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    dvl_seapilot::Measurement measurement;
    driver.readMeasurement(measurement);
    rbs.time = base::Time::now();
    rbs.velocity[0] = measurement.bottom_x;
    rbs.velocity[1] = measurement.bottom_y;
    rbs.velocity[2] = measurement.bottom_z;

//    std::cout << "HEADING: " << measurement.bottom_heading << std::endl;
//    std::cout << "PITCH:   " << measurement.bottom_pitch << std::endl;
//    std::cout << "ROLL:    " << measurement.bottom_roll << std::endl;

    

    _velocity_samples.write(rbs);


}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
