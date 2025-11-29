#pragma once
#include "posture_detection.h"

void connectToBlynk();
void sendToBlynk(PostureState state, float angle);
