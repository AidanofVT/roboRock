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
float slickness {0.8}; // An inverted friction value: determines how quickly the actuator stops when no forces are applied
float inertia {1}; // Determines how reluctantly the actuator accelerates. No real limits to this value.
float inRange {0.8}; // IMPORTANT: this is based on the known range of input voltages. If the voltage range changes, this should change.
float zeroMagnetism {0.01}; // Based on the known input signal noise.
float decelerationFactor {3}; // As input approaches zero, output approaches zero exponentially faster, according to this.
float inOffsetFromZero {};
float inZero {};
float inMax {};
float inMin {};
float inScaled {};
float outMin {0.0};
float outMax {1.0};
float maxSpeed {0.5};// {0.5}; // Per datasheet: max speed 33 inches per second.
float maxAcceleration {.002}; // {0.055}; // Can fully actuate in ~300ms. Though note that it's accelerating for the first and last ~100ms.
float velocity{0};
float deltaV{};
float command {}; // Allows us more precision in our calculations than AnalogOut allows. Actually does matter.
bool fromMasterPrior {false};
int debugOut{};

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
    float xa = 1.7 + copysign(1.3, inScaled * velocity + 0.000001);
    float xb = pow(abs(inScaled), xa) / inertia;
    float xc = copysign(xb, inScaled);
    float xd = clamp(xc, -maxAcceleration, maxAcceleration);
// pow() can't handle negative numbers raised to non-integer powers    // deltaV = clamp(copysign(pow(abs(inScaled) / inertia, 1.7 + copysign(1.3, inScaled * velocity)), inScaled), -maxAcceleration, maxAcceleration);
    // deltaV = clamp(pow(abs(inScaled) / inertia, decelerationFactor) * copysign(1, inScaled), -maxAcceleration, maxAcceleration);
    // if (Kernel::get_ms_count() % 200 == 0) {
    //     printf("%f, %f, %f \n", inScaled, velocity, xa);
    // }
    velocity = clamp(velocity * slickness + xd, -maxSpeed, maxSpeed);
    toActuator = command = clamp(command + velocity, outMin, outMax);
    if (command >= outMax || command <= outMin) {
        velocity = 0;
    }
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
    float startFrom = command;
    int moveStartTime = Kernel::get_ms_count();
    while (true) {
        readInputs();
        if ((inScaled > 0.15 || inScaled < -0.15) && yield == true) {
            printf("first escape\n");
            return false;
            break;
        }
        else if (pullToZero(command - to) == 0.0f) {
            printf("second escape\n");
            return true;
            break;
        }
        else {
            toActuator = command = lerp(moveStartTime, moveStartTime + duration, startFrom, to);
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
    outMin = command;
    // printf("outMin = %f\n", outMin);
    ThisThread::sleep_for(800ms);
    // printf("searching for bottom...\n");
    move(1.0, (command - 1.0) * -1 * 4000);
// Then repeat to get the bottom of the range.
    outMax = command;
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
    ThisThread::sleep_for(1500);
    inZero = fromAmp;
    inMax = inZero + inRange;
    inMin = inZero - inRange;
    printf("%f, %f, %f\n", inMin, inZero, inMax);
    calibrate();
    while (true) {
// If there's a rising edge in the 'retract now' signal, retract.
        if (fromMaster == true && fromMasterPrior == false) {
            move(outMin, 100, false);
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