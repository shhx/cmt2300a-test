#include <Arduino.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "radio/radio.h"

static uint8_t buf[17]; // Right now packet size its hardcoded to 17 bytes in src\radio\cmt2300a_params.h g_cmt2300aBasebandBank
static uint8_t cnt = 0;

void setup() {
    Serial.begin(115200);
    delay(100); // Wait for serial to initialize
    RF_Init();
    bool found = false;
    for(int i=0; i<5; i++) {
        if(CMT2300A_IsExist()) {
            Serial.println("CMT2300A Found!");
            found = true;
            break;
        }
        CMT2300A_DelayMs(10);
    }
    if(!found) {
        Serial.println("CMT2300A Not Found!");
        return;
    }
    strcpy((char *)buf, "Hello, CMT2300A!");
    memccpy(buf + 16, &cnt, 0, 1);
    RF_StartTx(buf, sizeof(buf), 1000);
}

void loop() {
    EnumRFResult result = RF_Process();
    if (result == RF_TX_DONE) {
        Serial.println("Transmission Completed!");
    } else if (result == RF_ERROR) {
        Serial.println("RF Error occurred!");
    } else if (result == RF_BUSY) {
        Serial.println("RF is busy...");
    } else if (result == RF_IDLE) {
        cnt++;
        delay(1000);
        Serial.print("Starting Transmission #");
        Serial.println(cnt);
        memccpy(buf + 16, &cnt, 0, 2);
        RF_StartTx(buf, sizeof(buf), 1000);
    } else {
        if (result != RF_IDLE) {
            Serial.println(RF_ResultToStr(result));
        }
    }
    delay(10);
}
