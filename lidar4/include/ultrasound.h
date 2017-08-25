enum ultrasound_sensors {
    FRONT_LEFT_DOWN   =  0,
    FRONT_LEFT_FRONT  =  1,
    FRONT_LEFT_LEFT   =  2,
    FRONT_RIGHT_DOWN  =  3,
    FRONT_RIGHT_FRONT =  4,
    FRONT_RIGHT_RIGHT =  5,
    REAR_LEFT_DOWN    =  6,
    REAR_LEFT_REAR    =  7,
    REAR_LEFT_LEFT    =  8,
    REAR_RIGHT_DOWN   =  9,
    REAR_RIGHT_REAR   = 10,
    REAR_RIGHT_RIGHT  = 11
};

void ultrasound_init (void);
int ultrasound_process (void); // Returns non-zero, if a new set of distances are available
double ultrasound_distance (int sensor); // Returns distance in meters
