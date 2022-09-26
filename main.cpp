#include "AnalogIn.h"
#include "DigitalIn.h"
#include "DigitalOut.h"
#include "Kernel.h"
#include "PinNames.h"
#include "ThisThread.h"
#include "mbed.h"
#include "mbed_thread.h"
#include <cstdio>
#include <ctime>
#include <utility>

AnalogIn fromAmp (p20);
AnalogOut toActuator (p18);
DigitalIn fromMaster (p19);
// DigitalOut stimulus (p23);
// PwmOut tone (p22);
float slickness {0.1};
float inertia {60};
float inRange {0.1};
float zeroMagnetism{0.03};
float inOffsetFromZero {};
float inZero {};
float inMax {};
float inMin {};
float inScaled {};
float outMin {0.0};
float outMax {1.0};
float velocity{0};
bool fromMasterPrior {false};
float debugOut{};

float clamp (float toClamp, float min, float max) {
    if (toClamp > max) {
        toClamp = max;
    }
    else if (toClamp < min) {
        toClamp = min;
    }
    return toClamp;
}

int signOf (float input) {
    return input / -input * -1;
}

float pullToZero (float toRound) {
    if (-zeroMagnetism < toRound && toRound < zeroMagnetism) {
        return 0;
    }
    else return toRound;
}

float readInputs () {
    inOffsetFromZero = clamp(fromAmp, inMin, inMax) - inZero;
    inScaled = pullToZero(inOffsetFromZero / inRange);
    return inScaled;
}

void comply () {
    readInputs();
    // if (inScaled != 0.0) {
        // printf("Input = %f &&& Velocity = %f\n", inScaled, velocity);
    // }
    velocity = clamp (velocity * slickness + inScaled / inertia, -1.0, 1.0);
    toActuator = clamp (toActuator + velocity, outMin, outMax);
}

float lirp (int startTime, int endTime, float startValue, float endValue) {
    float outPut{(float)(Kernel::get_ms_count() - startTime) / (endTime - startTime) * (endValue - startValue) + startValue};
    // printf("startTime = %i, endTime = %i, startValue = %f, endValue = %f, result: %f \n", startTime, endTime, startValue, endValue, outPut);
    return outPut;
}

// void siren () {
//     tone.period_us(1000 - Kernel::get_ms_count() % 50 * 10);
// }

bool move (float to, int duration, bool yield = true) {
    velocity = 0.0;
    float startFrom  = toActuator;
    int moveStartTime = Kernel::get_ms_count();
    while (true) {
        readInputs();
        if ((inScaled > 0.1 || inScaled < -0.1) && yield == true) {
            printf("first escape\n");
            return false;
            break;
        }
        else if (pullToZero(toActuator - to) == 0.0f) {
            printf("second escape\n");
            return true;
            break;
        }
        else {
            toActuator = lirp(moveStartTime, moveStartTime + duration, startFrom, to);
            ThisThread::sleep_for(1ms);
        }
    }
}

void calibrate () {
    printf("calibrating \n");
    // move(0.0, 1500);

    printf("searching for top...\n");
    move(1.0, 4000);
    outMin = toActuator;
    printf("outMin = %f\n", outMin);
    ThisThread::sleep_for(500ms);

    printf("searching for bottom...\n");
    move(1.0, (toActuator - 1.0) * -1 * 4000);
    outMax = toActuator;
    printf("outMax = %f\n", outMax);

    printf("returning to minimum...\n");
    move(outMin, 1000);
}

int main() {
    ThisThread::sleep_for(1200);
    inZero = fromAmp;
    inMax = inZero + inRange;
    inMin = inZero - inRange;
    printf("%f, %f, %f\n", inMin, inZero, inMax);
    calibrate();
    while (true) {
        // if (Kernel::get_ms_count() % 200 == 0) {
        //     printf("%f\n", readInputs());
        // }
        // if (Kernel::get_ms_count() % 750 < 100) {            
        //     stimulus = true;
        // }
        // else {
        //     stimulus = false;
        // }
        if (fromMaster == true && fromMasterPrior == false) {
            // printf("Beep!");
            move(outMin, 200, false);
            while (fromMaster == true) {
                ThisThread::sleep_for(1);
            }
        }
        else {
            comply();
        }
        fromMasterPrior = fromMaster;
        ThisThread::sleep_for(1ms);
    }
}