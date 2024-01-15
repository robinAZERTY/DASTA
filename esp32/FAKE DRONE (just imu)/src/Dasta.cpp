#include "Dasta.hpp"

SensorPreProcessing Dasta::sensors;
Actuators Dasta::actuators;

Dasta::Dasta()
{
    // allocate and link the matrices and vectors
    (sensors.acc_bias = Vector(3)).fill(0);
    (sensors.gyro_bias = Vector(3)).fill(0);
    (sensors.mag_bias = Vector(3)).fill(0);

    (sensors.acc_scale = Matrix(3, 3)).set_eye();
    (sensors.gyro_scale = Matrix(3, 3)).set_eye();
    (sensors.mag_scale = Matrix(3, 3)).set_eye();

    sensors.gyro.data = estimator.ekf->u->data;
    sensors.acc.data = estimator.ekf->u->data + 3;
    sensors.mag = Vector(3);

    configCommunication();
}

void Dasta::runDecisionOnUserEvent()
{
    switch (decisionnal_unit.user_event)
    {
    case UserEvent::None:
        break;
    case UserEvent::StartStateEstimate:
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
    case UserEvent::EnableStateEstimateStream:
        communication.send_stream.enable("position");
        communication.send_stream.enable("velocity");
        communication.send_stream.enable("orientation");
        break;
    case UserEvent::DisableStateEstimateStream:
        communication.send_stream.disable("position");
        communication.send_stream.disable("velocity");
        communication.send_stream.disable("orientation");
        break;
    case UserEvent::EnableSensorStream:
        communication.send_stream.enable("acc");
        communication.send_stream.enable("gyro");
        communication.send_stream.enable("mag");
        break;
    case UserEvent::DisableSensorStream:
        communication.send_stream.disable("acc");
        communication.send_stream.disable("gyro");
        communication.send_stream.disable("mag");
        break;
    default:
        break;
    }

    decisionnal_unit.user_event = UserEvent::None;

}