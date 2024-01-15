#ifndef DECISIONNAL_UNIT_CONFIG_HPP
#define DECISIONNAL_UNIT_CONFIG_HPP

enum UserEvent
{
    None,
    StartStateEstimate,
    StopStateEstimate,
    StartStream,
    StopStream,
    EnableStateEstimateStream,
    DisableStateEstimateStream,
    EnableSensorStream,
    DisableSensorStream,
};

#endif // DECISIONNAL_UNIT_CONFIG_HPP