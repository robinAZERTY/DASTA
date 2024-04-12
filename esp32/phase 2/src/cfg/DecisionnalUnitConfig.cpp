#include "Dasta.hpp"

enum UserEvent : uint8_t
{
    None=0,
    StartStateEstimate,
    StopStateEstimate,
    StartStream,
    StopStream,
    EnableStateEstimateStream,
    DisableStateEstimateStream,
    EnableSensorStream,
    DisableSensorStream,
    StartGyroBiasEstimation,
    StopGyroBiasEstimation,
    StartAttitudeControl,
    StopAttitudeControl,
    EMERGENCY_STOP,
    engageMotors,
    disengageMotors
};

enum EmbeddedEvent : uint8_t
{
    None2=0,
    WaitingForBatteryPower,
    WaitingForCalibration,
    WaitingForKalmanConvergence,
    ReadyToFly,
    LowBattery,
};


void Dasta::runDecisionOnUserEvent()
{
    switch (decisionnal_unit.user_event)
    {
    case UserEvent::None:
        break;
    case UserEvent::StartStateEstimate:
        estimator.initFromAccel(sensors.acc);
        estimator.running = true;
        break;
    case UserEvent::StopStateEstimate:
        estimator.running = false;
        break;
    case UserEvent::StartStream:
        communication.running_send_stream = true;
        break;
    case UserEvent::StopStream:
        communication.running_send_stream = false;
        break;
    case UserEvent::StartAttitudeControl:
        
        attitude_control_running = true;
        angular_velocity_control_running = true;
        break;
    case UserEvent::StopAttitudeControl:
        attitude_control_running = false;
        angular_velocity_control_running = false;
        break;

    // case UserEvent::EnableStateEstimateStream:
    //     communication.send_stream.enable("position");
    //     communication.send_stream.enable("velocity");
    //     communication.send_stream.enable("orientation");
    //     break;
    // case UserEvent::DisableStateEstimateStream:
    //     communication.send_stream.disable("position");
    //     communication.send_stream.disable("velocity");
    //     communication.send_stream.disable("orientation");
    //     break;
    case UserEvent::EnableSensorStream:
        communication.send_stream.enable("acc");
        communication.send_stream.enable("gyro");
        // communication.send_stream.enable("mag");
        break;
    case UserEvent::DisableSensorStream:
        communication.send_stream.disable("acc");
        communication.send_stream.disable("gyro");
        // communication.send_stream.disable("mag");
        break;
    case UserEvent::StartGyroBiasEstimation:
        sensors.startGyroBiasEstimation();
        break;
    case UserEvent::StopGyroBiasEstimation:
        sensors.gyro_bias_estimation_running = false;
        break;
    case UserEvent::EMERGENCY_STOP:
        actuators.stopMotors();
        ESP.restart();
        break;
    case UserEvent::engageMotors:
        sensors.LiPo.read();
        if (sensors.LiPo.voltages.data[3]<6)
            decisionnal_unit.internal_event = EmbeddedEvent::WaitingForBatteryPower;
        else
        {
        actuators.engageMotors();
        attitude_control_running = true;
        angular_velocity_control_running = true;
        pidRx.reset_integrale();
        pidRy.reset_integrale();
        pidRz.reset_integrale();
        }
        break;
    case UserEvent::disengageMotors:
        actuators.stopMotors();
        attitude_control_running = false;
        angular_velocity_control_running = false;
        break;

    default:
        break;
    }

    decisionnal_unit.user_event = UserEvent::None;

}