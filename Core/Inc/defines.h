#ifndef __DEFINES_H
#define __DEFINES_H

// Message IDs
#define CAN_MOTOR_DATA_ID 	   0x010
#define CAN_LIGHT_CONTROL_ID   0x011
#define CAN_BLINK_CONTROL_ID   0x012
#define CAN_SENSOR_DATA_ID     0x030
#define CAN_TEST_MSG_ID        0x050
#define CAN_DEVICE_SETTINGS_ID 0x060

enum sensors {
	SENSOR_HCSR,
	SENSOR_VL53,
};

enum board_position {
    BACK_RIGHT,
    FRONT_RIGHT,
    BACK_LEFT,
    FRONT_LEFT,
};

#endif
