#include "blynk_integration.h"

#ifdef USE_BLYNK
#include <BlynkSimpleEsp32.h>

char auth[] = "YOUR_BLYNK_TOKEN";
char ssid[] = "YOUR_WIFI";
char pass[] = "YOUR_PASS";

void connectToBlynk()
{
    Blynk.begin(auth, ssid, pass);
}

void sendToBlynk(PostureState state, float angle)
{
    Blynk.virtualWrite(V1, angle);
    Blynk.virtualWrite(V0, (int)state);
}

#endif
