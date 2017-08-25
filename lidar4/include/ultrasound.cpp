#include <wiringPi.h>
#include <time.h>
#include <stdio.h>
#include "ultrasound.h"

#define PIN_TRIG                   25
#define PIN_ECHO_FRONT_LEFT_DOWN   16
#define PIN_ECHO_FRONT_LEFT_FRONT   7
#define PIN_ECHO_FRONT_LEFT_LEFT    8
#define PIN_ECHO_FRONT_RIGHT_DOWN  26
#define PIN_ECHO_FRONT_RIGHT_FRONT 20
#define PIN_ECHO_FRONT_RIGHT_RIGHT 21
#define PIN_ECHO_REAR_LEFT_DOWN     9
#define PIN_ECHO_REAR_LEFT_REAR    11
#define PIN_ECHO_REAR_LEFT_LEFT    10
#define PIN_ECHO_REAR_RIGHT_DOWN   22
#define PIN_ECHO_REAR_RIGHT_REAR   27
#define PIN_ECHO_REAR_RIGHT_RIGHT  17

#define ULTRASOUND_TRIG_TIME 20e3 // nanoseconds
#define ULTRASOUND_MEASUREMENT_CYCLE 100e6 // nanoseconds

enum ultrasound_sensor_states {
    ULTRASOUND_SENSOR_IDLE,
    ULTRASOUND_SENSOR_WAITING_FOR_RISING_EDGE,
    ULTRASOUND_SENSOR_WAITING_FOR_FALLING_EDGE,
    ULTRASOUND_SENSOR_ECHO_RECEIVED
};

enum ultrasound_module_states {
    ULTRASOUND_IDLE,
    ULTRASOUND_TRIGGING,
    ULTRASOUND_WAITING_FOR_ECHOS,
    ULTRASOUND_WAITING_FOR_CYCLE
};

int ultrasound_echo_pin[12];
int ultrasound_sensor_state[12];
int ultrasound_module_state = ULTRASOUND_IDLE;
struct timespec ultrasound_trig_time;
struct timespec ultrasound_echo_rising_edge[12];
struct timespec ultrasound_echo_falling_edge[12];
double ultrasound_distance_var[12]; // meters

void ultrasound_init (void)
{
    ultrasound_echo_pin[0] = PIN_ECHO_FRONT_LEFT_DOWN;
    ultrasound_echo_pin[1] = PIN_ECHO_FRONT_LEFT_FRONT;
    ultrasound_echo_pin[2] = PIN_ECHO_FRONT_LEFT_LEFT;
    ultrasound_echo_pin[3] = PIN_ECHO_FRONT_RIGHT_DOWN;
    ultrasound_echo_pin[4] = PIN_ECHO_FRONT_RIGHT_FRONT;
    ultrasound_echo_pin[5] = PIN_ECHO_FRONT_RIGHT_RIGHT;
    ultrasound_echo_pin[6] = PIN_ECHO_REAR_LEFT_DOWN;
    ultrasound_echo_pin[7] = PIN_ECHO_REAR_LEFT_REAR;
    ultrasound_echo_pin[8] = PIN_ECHO_REAR_LEFT_LEFT;
    ultrasound_echo_pin[9] = PIN_ECHO_REAR_RIGHT_DOWN;
    ultrasound_echo_pin[10] = PIN_ECHO_REAR_RIGHT_REAR;
    ultrasound_echo_pin[11] = PIN_ECHO_REAR_RIGHT_RIGHT;
    for (int n=0; n<12; ++n) {
        pinMode(ultrasound_echo_pin[n], INPUT);
        ultrasound_sensor_state[n] = ULTRASOUND_SENSOR_IDLE;
    }
    pinMode(PIN_TRIG, OUTPUT);
}

long ultrasound_time_difference (struct timespec *start, struct timespec *stop)
{
    if (stop->tv_nsec > start->tv_nsec) {
        return stop->tv_nsec - start->tv_nsec;
    } else {
        // The nanosecond counter has wrapped around
        return 1e9 - start->tv_nsec + stop->tv_nsec;
    }
}

// Returns non-zero, if a new set of distances are available
int ultrasound_process (void)
{
    struct timespec temp;
    long delta;
    
    /* Debug output
    printf("ultrasound_module_state: %d\n", ultrasound_module_state);
    printf("ultrasound_sensor_state[n]:\n");
    for (int n=0; n<12; ++n) {
        printf("%d", ultrasound_sensor_state[n]);
        if (n != 11) {
            printf(", ");
        } else {
            printf("\n");
        }
    }*/
    
    for (int n=0; n<12; ++n) {
        switch (ultrasound_sensor_state[n]) {
            case ULTRASOUND_SENSOR_IDLE:
                // Do nothing
            break;
            case ULTRASOUND_SENSOR_WAITING_FOR_RISING_EDGE:
                if (digitalRead(ultrasound_echo_pin[n]) == HIGH) {
                    clock_gettime(CLOCK_REALTIME, &ultrasound_echo_rising_edge[n]);
                    ultrasound_sensor_state[n] = ULTRASOUND_SENSOR_WAITING_FOR_FALLING_EDGE;
                }
            break;
            case ULTRASOUND_SENSOR_WAITING_FOR_FALLING_EDGE:
                if (digitalRead(ultrasound_echo_pin[n]) == LOW) {
                    clock_gettime(CLOCK_REALTIME, &ultrasound_echo_falling_edge[n]);
                    ultrasound_sensor_state[n] = ULTRASOUND_SENSOR_ECHO_RECEIVED;
                }
            break;
            case ULTRASOUND_SENSOR_ECHO_RECEIVED:
                // Do nothing
            break;
        };
    }
    switch (ultrasound_module_state) {
        case ULTRASOUND_IDLE:
            // Record current time and set trig signal high
            clock_gettime(CLOCK_REALTIME, &ultrasound_trig_time);
            digitalWrite(PIN_TRIG, HIGH);
            ultrasound_module_state = ULTRASOUND_TRIGGING;
        break;
        case ULTRASOUND_TRIGGING:
            // If enough time has passed, set trig low
            clock_gettime(CLOCK_REALTIME, &temp);
            if (ultrasound_time_difference(&ultrasound_trig_time, &temp) > ULTRASOUND_TRIG_TIME) {
                for (int n=0; n<12; ++n) {
                    ultrasound_sensor_state[n] = ULTRASOUND_SENSOR_WAITING_FOR_RISING_EDGE;
                }
                digitalWrite(PIN_TRIG, LOW);
                ultrasound_module_state = ULTRASOUND_WAITING_FOR_ECHOS;
            }
        break;
        case ULTRASOUND_WAITING_FOR_ECHOS:
            for (int n=0; n<12; ++n) {
                if (ultrasound_sensor_state[n] != ULTRASOUND_SENSOR_ECHO_RECEIVED) {
                    return 0;
                }
            }
            // Echo has been received for all sensors. Recalculate distances
            for (int n=0; n<12; ++n) {
                delta = ultrasound_time_difference(&ultrasound_echo_rising_edge[n], &ultrasound_echo_falling_edge[n]);
                ultrasound_distance_var[n] = ((double) delta) * 340 / 1e9 / 2;
                ultrasound_sensor_state[n] = ULTRASOUND_SENSOR_IDLE;
            }
            ultrasound_module_state = ULTRASOUND_WAITING_FOR_CYCLE;
            return 1;
        break;
        case ULTRASOUND_WAITING_FOR_CYCLE:
            // Wait for measurement cycle to complete
            clock_gettime(CLOCK_REALTIME, &temp);
            if (ultrasound_time_difference(&ultrasound_trig_time, &temp) > ULTRASOUND_MEASUREMENT_CYCLE) {
                ultrasound_module_state = ULTRASOUND_IDLE;
            }
        break;
    };
    return 0;
}

// Returns distance in meters
double ultrasound_distance (int sensor)
{
    return ultrasound_distance_var[sensor];
}
