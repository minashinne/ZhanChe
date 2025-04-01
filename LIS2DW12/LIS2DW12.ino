#include <LIS2DW12Sensor.h>

#ifdef ARDUINO_SAM_DUE
#define DEV_I2C Wire1
#elif defined(ARDUINO_ARCH_STM32)
#define DEV_I2C Wire
#elif defined(ARDUINO_ARCH_AVR)
#define DEV_I2C Wire
#else
#define DEV_I2C Wire
#endif
#define SerialPort Serial


void setup() {
LIS2DW12Sensor Acc2(&DEV_I2C, LIS2DW12_I2C_ADD_H);
}

void loop() {
  // put your main code here, to run repeatedly:

}
