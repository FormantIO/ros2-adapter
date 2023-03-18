from rclpy.qos import (
    qos_profile_unknown,
    qos_profile_system_default,
    qos_profile_sensor_data,
    qos_profile_services_default,
    qos_profile_parameters,
    qos_profile_parameter_events,
    qos_profile_action_status_default,
)

QOS_PROFILES = {
    "UNKNOWN": qos_profile_unknown,
    "SYSTEM_DEFAULT": qos_profile_system_default,
    "SENSOR_DATA": qos_profile_sensor_data,
    "SERVICES_DEFAULT": qos_profile_services_default,
    "PARAMETERS": qos_profile_parameters,
    "PARAMETER_EVENTS": qos_profile_parameter_events,
    "ACTION_STATUS_DEFAULT": qos_profile_action_status_default,
}
