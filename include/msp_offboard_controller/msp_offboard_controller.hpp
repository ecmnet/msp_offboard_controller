
#define OFFBOARD_RATE 25

#define MIN_YAW_FOLLOW_VELOCITY 0.30f

#define MSG_PREFIX "/msp"

#define MSP_CMD_PUB  MSG_PREFIX "/in/vehicle_command"
#define MSP_LOG_PUB  "/msp/in/log_message"
#define MSP_TRJ_PUB  MSG_PREFIX "/in/trajectory_setpoint"

#define MSP_POS_SUB     MSG_PREFIX "/out/vehicle_local_position"
#define MSP_STATUS_SUB  MSG_PREFIX "/out/vehicle_status"
#define MSP_CMD_SUB                "/msp/out/vehicle_command"

#include <msp_controller/msp_px4.h>

