#include <Arduino.h>

#if !defined(ARDUINO_ESP32C3_DEV)  // This is due to a bug in RISC-V compiler, which requires unused function sections :-(.
#define DISABLE_CODE_FOR_RECEIVER  // Disables static receiver code like receive timer ISR handler and static IRReceiver and irparams data. Saves 450 bytes program memory and 269 bytes RAM if receiving functions are not required.
#endif
// #define SEND_PWM_BY_TIMER         // Disable carrier PWM generation in software and use (restricted) hardware PWM.
// #define USE_NO_SEND_PWM           // Use no carrier PWM, just simulate an active low receiver signal. Overrides SEND_PWM_BY_TIMER definition

#include <IRremote.hpp>  // include the library

#define IR_SEND_PIN             PB15
#define IR_SEND_PIN_STRING      "PB15"
//ir nec开头修改数据长度并添加到IRsend::sendNECRaw中，同时修改IRsend::sendNECRaw的数据长度为uint64，并同步修改irremote中的预定义处的类型。

String cmd;
void setup() {

  Serial.begin(115200);
  IrSender.begin(IR_SEND_PIN);      // Start with IR_SEND_PIN -which is defined in PinDefinitionsAndMore.h- as send pin and enable feedback LED at default feedback LED pin
  disableLEDFeedback();  // Disable feedback LED at default feedback LED pin
  delay(1000);
}

uint64_t MAC = 0;
void loop() {
  IrSender.sendNECRaw(0x1C9DC245BB3C, 0);
  delay(100);  // delay must be greater than 5 ms (RECORD_GAP_MICROS), otherwise the receiver sees it as one long signal
}


