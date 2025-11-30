#include <Arduino.h>
#include "imu_driver.h"

IMUDriver imu;

void setup()
{
    Serial.begin(115200);
    imu.begin();
    delay(200);
    Serial.println("START_CSV");
    Serial.println("ax,ay,az,gx,gy,gz,mx,my,mz");
}

void loop()
{
    IMUSample s;
    if (imu.readSample(s))
    {
        Serial.print(s.ax, 6);
        Serial.print(',');
        Serial.print(s.ay, 6);
        Serial.print(',');
        Serial.print(s.az, 6);
        Serial.print(',');
        Serial.print(s.gx, 6);
        Serial.print(',');
        Serial.print(s.gy, 6);
        Serial.print(',');
        Serial.print(s.gz, 6);
        Serial.print(',');
        Serial.print(s.mx, 6);
        Serial.print(',');
        Serial.print(s.my, 6);
        Serial.print(',');
        Serial.println(s.mz, 6);
    }
    delay(SAMPLE_PERIOD_MS);
}
