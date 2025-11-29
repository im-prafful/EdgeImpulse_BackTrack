#include <Wire.h>
#include "MPU9250.h"

MPU9250 mpu;

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    mpu.setup(0x68);
    delay(1000);
}

void loop()
{
    mpu.update();
    Serial.print(mpu.getAccX());
    Serial.print(",");
    Serial.print(mpu.getAccY());
    Serial.print(",");
    Serial.print(mpu.getAccZ());
    Serial.print(",");
    Serial.print(mpu.getGyroX());
    Serial.print(",");
    Serial.print(mpu.getGyroY());
    Serial.print(",");
    Serial.println(mpu.getGyroZ());
    delay(20);
}
