extern bool gamepad_mounted;

#define WHEEL_SENSOR_MAX 65535

typedef struct
{    
  uint8_t x, y, z, rz; // joystick

  uint8_t l2, l3, r2, r3; // buttons

  uint16_t ruddle, throttle, brake;

} gamepad_state_t;

extern gamepad_state_t gamepad_state;