// Wheel Functions
#include <esp_attr.h>

typedef struct {
    float kp, ki, kd;
} PIDConfig;

void wheel_init();
void wheel_setPID(PIDConfig cfg);
void wheel_run();
void setLeftTarget(float target);
void setRightTarget(float target);
