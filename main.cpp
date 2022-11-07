#include "AnalogIn.h"
#include "DigitalIn.h"
#include "DigitalOut.h"
#include "Kernel.h"
#include "PinNames.h"
#include "ThisThread.h"
#include "mbed.h"
#include "mbed_thread.h"
#include <cmath>
#include <cstdio>
#include <ctime>
#include <utility>

AnalogIn fromAmp (p20);
AnalogOut toActuator (p18);
DigitalIn fromMaster (p19);
float slickness {0.999}; // An inverted friction value: determines how quickly the actuator stops when no forces are applied
float inertia {0.75}; // Determines how reluctantly the actuator accelerates. No real limits to this value.
int predictXCyclesAhead {20}; // Controls the relative strength of the 'derivative' portion of the comply() algorithm.
float inRange {0.3}; // IMPORTANT: this is based on the known range of input voltages. If the voltage range changes, this should change.
float inZero {}; // Based on value at startup
float inMax {};
float inMin {};
float inScaled {};
float inScaledPrior {};
float outMin {0.0};
float outMax {1.0};
float maxSpeed {0.0055};     // Per datasheet: max speed 33 inches per second.
float maxAcceleration {0.0003}; // Can fully actuate in ~300ms. Note, though, that it's accelerating for the first and last ~100ms.
                                // IMPORTANT: acceleration is also proportional to this!
float velocity{0};
float command {}; // Allows us more precision in our calculations than AnalogOut allows. Actually does matter.
float anticipatedAUC; // AUC = Area Under Curve

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

float lerp (int startTime, int endTime, float startValue, float endValue) {
/*
"lerp" is a terrible abreviation of "linear interpolation."
Given two ranges, one of two times, and another of two arbitrary values, returns a value between 
the two arbitrary values, proportional to the current time on the range between the two times given.
*/
    return (float)(Kernel::get_ms_count() - startTime) / (endTime - startTime) * (endValue - startValue) + startValue;
}

float readInputs () {
// Updates the 'inScaled' variable, a 0-1 clamped representation of the force signal.
    inScaledPrior = inScaled;
    inScaled = (clamp(fromAmp, inMin, inMax) - inZero) / inRange;
    return inScaled;
}

float specialSauce (float input) {  
    // Just changes the number provided depending on whether or not it opposes the current velocity.  
    static float factor {};
    factor = 5.5 + copysign(4.5, input * velocity + 0.00000001);
    // Leaving this here for future reference. pow() can't handle negative numbers raised to non-integer powers:
    // return copysign(pow(abs(input), factor) / factor, input);
    return input / factor;
}

float calculateFutureAUC () {
/* AUC = "area under curve".
It's one part the current inScaled, three parts the previous inScaled, and twenty parts articipated future inScaled values.
Future values simply assume the current rate of change. */
    float slope = inScaled - inScaledPrior;
    float point = inScaled;
    anticipatedAUC = specialSauce(inScaledPrior) * 3;
    for (int i = predictXCyclesAhead + 1; i > 0; --i) {
        anticipatedAUC += specialSauce(point);
        point += slope;
    }
    // if (slope != 0.0 && Kernel::get_ms_count() % 1500 == 0) {
    //     printf("%f, %f, %f, %f, %f \n", inScaled, velocity, slope, point, anticipatedAUC);
    // }
    return anticipatedAUC;
}

void comply () {
/* This is the important part: where the (imaginary/prescriptive) velocity is calculated, and the actuator is commanded.
This function is the only content of the main loop, as long as it's not executing a move command.
If you want to change how the actuator floats, it's probably going to be done here. */
    static float rawDeltaV {};
    static float deltaV {};
    calculateFutureAUC();
    // pow(predictXCyclesAhead, 2) is the theoretical maximum AUC.
    rawDeltaV = anticipatedAUC / pow(predictXCyclesAhead, 2);
    // Velocity is a component here because if friction (slickness) acts proportionally to speed, force should too.
    deltaV = clamp(rawDeltaV * (abs(velocity) + 0.007) / inertia, -maxAcceleration, maxAcceleration);
    // if (Kernel::get_ms_count() % 200 == 0) {
    //     printf("%f, %f, %f, %f, %f \n", inScaledPrior, inScaled, velocity, anticipatedAUC, rawDeltaV);
    // }
    float provisionalVelocity = clamp(velocity * slickness + deltaV, -maxSpeed, maxSpeed);
    velocity = provisionalVelocity;
    toActuator = command = clamp(command + velocity, outMin, outMax);
    // If the actuator could have velocity-debt while stuck on the end if its range, that would be bad:
    if (command >= outMax || command <= outMin) {
        velocity = 0;
    }
}

bool move (float to, int duration, bool yield = true) {
/*
Moves the actuator to position "to" over "duration" milliseconds.
By default, the movement will yield to even a small resistance, meaning it's not intended for use under load.
If yield = false, though, the movement will be forced.
*/
    velocity = 0.0;
    float startFrom = command;
    int moveStartTime = Kernel::get_ms_count();
    while (true) {
        readInputs();
        if ((inScaled > 0.15 || inScaled < -0.15) && yield == true) {
            // printf("Movement ended; encountered resistance.\n");
            return false;
            break;
        }
        // "If the destination has been nearly reached, or passed."
        else if ((to - command) * copysign(1, to - startFrom) < 0.01) {
            // printf("Movement ended; destination reached.\n");
            return true;
            break;
        }
        else {
            toActuator = command = lerp(moveStartTime, moveStartTime + duration, startFrom, to);
            ThisThread::sleep_for(1ms);
        }
    }
}

void insertForce (float force) {
// Adding force to the inScaled variable artificially causes comply() to push/pull with that much force.
    inScaled = clamp(inScaled + force, -1.0, 1.0);
}

void calibrate () {
    // This initial delay is to let any physical shaking work itself out before an initial measurement is taken.
    ThisThread::sleep_for(1500);
    inZero = fromAmp;
    inMax = inZero + inRange;
    inMin = inZero - inRange;
    // Starting from the minimum position, moves the actuator slowly downward...
    move(1.0, 4000);
    /* --until some significant resistance is detected. The current positions becomes the top of the working range.
    The intention is for a user to use place their hand where they want the limit to be.*/
    outMin = command;
    // printf("outMin = %f\n", outMin);
    ThisThread::sleep_for(800ms);
    move(1.0, (command - 1.0) * -1 * 4000);
    // Then repeat to get the bottom of the range.
    outMax = command;
    // printf("outMax = %f\n", outMax);
    move(outMin, 1000);
    printf("%f, %f, %f ... %f, %f\n", inMin, inZero, inMax, outMin, outMax);
}

int main() {
/*
If a user wants to re-define movement limits, or recalibrate input, they are expected to simply power-cycle the microcontroller.
This will cause the actuator to make some big, abrupt moves though. 
*/
    calibrate();
    while (true) {
        readInputs();
        if (fromMaster == true) {
            insertForce(-0.5);
        }
        comply();
        ThisThread::sleep_for(1ms);
    }
}