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

void Task::processIO()
{
    driver->read();
    _device_info.set(driver->deviceInfo);
      
    // Compute the data timestamp
    base::Time base_time = base::Time::now();
    int64_t seq = driver->status.seq;
    if (last_seq < 0)
        global_seq = seq;
    else if (last_seq > seq) // wrapped around
        global_seq += static_cast<uint64_t>((1 << 24) - seq) + last_seq;
    else
        global_seq += seq - last_seq;
    last_seq = seq;
    base::Time time = timestamp_estimator->update(base_time, global_seq);

    // Update the timestamp on each of the fields, and write it on our outputs
    driver->status.time = time;
    _status.write(driver->status);

    if (!driver->cellReadings.time.isNull())
    {
        driver->cellReadings.time = time;
        _cell_samples.write(driver->cellReadings);
    }

    if (!driver->bottomTracking.time.isNull())
    {
        driver->bottomTracking.time = time;
        _bottom_tracking_samples.write(driver->bottomTracking);

        base::samples::RigidBodyState rbs_ground_distance;
        rbs_ground_distance.time = time;
        rbs_ground_distance.invalidate();
        if( (!base::isUnknown<float>(driver->bottomTracking.range[0])) &&
            (!base::isUnknown<float>(driver->bottomTracking.range[1])) &&
            (!base::isUnknown<float>(driver->bottomTracking.range[2])) &&
            (!base::isUnknown<float>(driver->bottomTracking.range[3])) )
        {
                //Taking the Average distance to the bottom if all readings are valid
                double avg = (driver->bottomTracking.range[0] +
                              driver->bottomTracking.range[1] +
                              driver->bottomTracking.range[2] +
                              driver->bottomTracking.range[3])/4.0;
                
                avg *= cos(20.0/180.0*M_PI); //20 degree angle of the pistons, convert to distance
                rbs_ground_distance.position[2] = avg;
                rbs_ground_distance.cov_position(2,2) = _variance_ground_distance.get();
        }
        //Write ground distance even we have no lock, then with NaN information
        _ground_distance.write(rbs_ground_distance);
    }
    else
    {
        RTT::log(RTT::Error) << "No DVL data received!" << RTT::endlog();
    }

    // Extract RigidBodyState data and write it to speed_samples
    //
    // This is possible in all but BEAM coordinate mode
    if (driver->outputConf.coordinate_system != dvl_teledyne::BEAM)
    {
        base::samples::RigidBodyState rbs_velocity;
        rbs_velocity.invalidate();
        rbs_velocity.time = time;

        //check for nans 
        if( !base::isUnknown(driver->bottomTracking.velocity[0]) &&
            !base::isUnknown(driver->bottomTracking.velocity[1]) &&
            !base::isUnknown(driver->bottomTracking.velocity[2]) &&
            !base::isUnknown(driver->bottomTracking.velocity[3]) )
        {
            // set variance
            double var = 1.0;
            if(_sigma.value() > 0.0)
                var = pow(_sigma.value(), 2.0);
            else
            {
                RTT::log(RTT::Warning) << "Sigma has to be a posstive value! Override it with 0.01" << RTT::endlog();
                var = pow(0.01, 2.0);
            }

            rbs_velocity.orientation  = driver->status.orientation;
            rbs_velocity.velocity.x() = driver->bottomTracking.velocity[0];
            rbs_velocity.velocity.y() = driver->bottomTracking.velocity[1];
            rbs_velocity.velocity.z() = driver->bottomTracking.velocity[2];
            rbs_velocity.cov_velocity = var * Eigen::Matrix3d::Identity();

            _velocity_samples.write(rbs_velocity);
        }
        else
        {
            //Write this command even we have no bottom-lock indicating it with NaN readings
            _velocity_samples.write(rbs_velocity);
        }
    }
    else
    {
        RTT::log(RTT::Warning) << "Cannot provide velocity samples in beam coordinate system." << RTT::endlog();
    }

    // write timestamp estimator status
     _timestamp_estimator_status.write(timestamp_estimator->getStatus());
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    driver.reset(new dvl_seapilot::Driver);
    driver->setEnsembleInterval(_ensemble_interval.value());
    driver->setReadTimeout(_io_read_timeout.value());
    driver->setWriteTimeout(_io_write_timeout.value());

    if (!_io_port.value().empty())
        driver->open(_io_port.value());

    setDriver(driver.get());

    timestamp_estimator.reset(new aggregator::TimestampEstimator(base::Time::fromSeconds(100),base::Time::fromSeconds(0.1)));

    if (! TaskBase::configureHook())
        return false;
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    driver->startAcquisition();
    timestamp_estimator->reset();
    last_seq = -1;

    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();

    driver->stopAcquisition();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    driver.reset();
}
