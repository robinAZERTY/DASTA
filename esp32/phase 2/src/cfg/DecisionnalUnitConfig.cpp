#include "Dasta.hpp"

enum UserEvent : uint8_t
{
    None = 0,
    EnableStream,
    DisableStream,
    SwitchToRaceMode,
    SwitchToAttiMode,
    SwitchToVelMode,
    SwitchToPosMode,
    EngageMotors,
    DisengageMotors,
    EMERGENCY_STOP
};

enum EmbeddedEvent : uint8_t
{
    None2 = 0,
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
    case UserEvent::EnableStream:
        communication.running_send_stream = true;
        break;
    case UserEvent::DisableStream:
        communication.running_send_stream = false;
        break;
    case UserEvent::SwitchToRaceMode:
        control_mode = ControlMode::ANGULAR_VELOCITY;
        break;
    case UserEvent::SwitchToAttiMode:
        control_mode = ControlMode::ATTITUDE;
        break;
    case UserEvent::SwitchToVelMode:
        control_mode = ControlMode::VELOCITY;
        break;
    case UserEvent::SwitchToPosMode:
        control_mode = ControlMode::POSITION;
        break;
    case UserEvent::EngageMotors:
        sensors.LiPo.read(this->now/1000000.0);
        if (sensors.LiPo.voltages.data[3]<6)
            decisionnal_unit.internal_event = EmbeddedEvent::WaitingForBatteryPower;
        else if(sensors.LiPo.charges.data[3]<0.10)
            decisionnal_unit.internal_event = EmbeddedEvent::LowBattery;
        else if (!sensors.calibrate)
            decisionnal_unit.internal_event = EmbeddedEvent::WaitingForCalibration;
        else if (estimator.get_orientation_max_cov()>0.05)
            decisionnal_unit.internal_event = EmbeddedEvent::WaitingForKalmanConvergence;
        else
        {
        decisionnal_unit.internal_event = EmbeddedEvent::ReadyToFly;
        actuators.engageMotors();
        pidRx.reset_integrale();
        pidRy.reset_integrale();
        pidRz.reset_integrale();
        estimator.set_rp_offset();
        }
        break;
    case UserEvent::DisengageMotors:
        actuators.stopMotors();
        break;
    case UserEvent::EMERGENCY_STOP:
        actuators.stopMotors();
        ESP.restart();
        break;
    }

    decisionnal_unit.user_event = UserEvent::None;
}