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
float slickness {0.1}; // An inverted friction value: determines how quickly the actuator stops when no forces are applied
float inertia {60}; // Determines how reluctantly the actuator accelerates. No real limits to this value.
float inRange {0.1}; // IMPORTANT: this is based on the known range of input voltages. If the voltage range changes, this should change.
float zeroMagnetism{0.03}; // Based on the known input signal noise. 
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
// Given a value, returns that value if it's within a given maximum / minimum.
// Otherwise, returns the violated maximum / minimum.
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
// Used for filtering out input noise, and simplifying float math when things approach zero.
// Note that it's not really a rounding function, since if it's not pulled to zero, precision is maintained.
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
/*
This is the important part: where the (imaginary/prescriptive) velocity is calculated, and the actuator is commanded.
This function is the only contexnt of the main loop, when it's not executing a move command.
If you want to change how the actuator behaves, it's probably going to be done here.
One possible improvement is to make velocity zero when the limits are reached.
*/
    readInputs();
    velocity = clamp (velocity * slickness + inScaled / inertia, -1.0, 1.0);
    toActuator = clamp (toActuator + velocity, outMin, outMax);
}

float lerp (int startTime, int endTime, float startValue, float endValue) {
/*
"lerp" is a terrible abreviation of "linear interpolation."
Given two ranges, one of two times, and another of two arbitrary values, returns a value between 
the two arbitrary values, proportional to the current time on the range between the two times given.
*/
    float outPut{(float)(Kernel::get_ms_count() - startTime) / (endTime - startTime) * (endValue - startValue) + startValue};
    return outPut;
}

bool move (float to, int duration, bool yield = true) {
/*
Moves the actuator to position "to" over "duration" milliseconds.
By default, the movement will yield to even a small resistance, meaning it's not intended for use under load.
If yield = false, though, the movement will be forced.
*/
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
            toActuator = lerp(moveStartTime, moveStartTime + duration, startFrom, to);
            ThisThread::sleep_for(1ms);
        }
    }
}


void calibrate () {
// Starting from the minimum position, moves the actuator slowly downward...
    // printf("calibrating \n");
    // printf("searching for top...\n");
    move(1.0, 4000);
/* --until some significant resistance is detected. The current positions becomes the top of the working range.
   The intention is for a user to use place their hand where they want the limit to be.*/
    outMin = toActuator;
    // printf("outMin = %f\n", outMin);
    ThisThread::sleep_for(500ms);
    // printf("searching for bottom...\n");
    move(1.0, (toActuator - 1.0) * -1 * 4000);
// Then repeat to get the bottom of the range.
    outMax = toActuator;
    // printf("outMax = %f\n", outMax);
    // printf("returning to minimum...\n");
    move(outMin, 1000);
}

int main() {
/*
Since the system has no inputs besides readings from the force transducer, there is no way to command a recalibration.
The intent is for users to simply power-cycle the controller when recalibration is needed.
This will cause the actuator to make some big, abrupt moves though. 
*/
// This initial delay is to let any physical shaking work itself out before an initial measurement is taken.
    ThisThread::sleep_for(1200);
    inZero = fromAmp;
    inMax = inZero + inRange;
    inMin = inZero - inRange;
    // printf("%f, %f, %f\n", inMin, inZero, inMax);
    calibrate();
    while (true) {
// If there's a rising edge in the 'retract now' signal, retract.
        if (fromMaster == true && fromMasterPrior == false) {
            move(outMin, 200, false);
//      Following a forced move, wait for the 'retract now' signal to disappear.
            while (fromMaster == true) {
                ThisThread::sleep_for(1);
            }
        }
// Otherwise, comply to outside forces.
        else {
            comply();
        }
        fromMasterPrior = fromMaster;
        ThisThread::sleep_for(1ms);
    }
}