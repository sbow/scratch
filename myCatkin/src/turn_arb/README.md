### > turn_arb package description

#### > Purpose:
* React to teleop [-1..1] and AV turn radius steering requests
* Provide command matrix to servo driver node - pca9685

### > Subscribed Topics:
* /teleop_logi/tele_steer_req | Gamepad steering, Float32, -1 (right)..1(left)
* av_turnrad_req | Autonomous vehicle control turn radius req (-right, +left)
* /teleop_logi/human_in_loop | Gamepad shows human is holding down dedman switch, bool

### > Published Topics:
* /servo/cmdVector | Vector of 16 integer values corresponding to desired duty for PWM

### > Hardware:
* servo | stock, Traxxas SLASH 4x4
* servo controller | Adafruit 16 Channel PWM Driver, pca9685 chip
* computer | tested on ROS Noetic & Kame, Arm64 & AMD64, NVidia Jetson TX1 & desktop

### > Tricks and hacks:
* Publish /servo/cmdVector type message for PWM driver at command line for testing - assuming servo plugged into channel 0:
```
rostopic pub /command std_msgs/In2MultiArray -r 10 "{layout: {dim: [{label: '', size: 0, stride: 0}], data_offset: 0}, data: [5100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"
```

### > Program flow:
* do includes, inc each message type
* do #define's for more clear code later
* declare variables to be populated later by params.yaml
* declare variables to be referenced by package defining servo state
* declare variables to be populated by input topics
* floatIsEq - Simple floating point equality test
* getNeighborsIx: This is part of linear interpolation of params.yaml vector parameters linking turn rad to servo PWM
It is given the axis to be used with the lookup table, and the input value to be looked up
It returns a vector of 3 elements length.
Element 0: number of "neighboring" elements. This will be either 1 (if input value beyond axis), or 2 if input value within axis
Element 1&2: the index corresponding to the neighboring axis values. These indicies are used to look-up the neighboring values from
the table, which are then used in a linear interpolation to determine the right lookup value for the input value.
* floatInterp1: This is the main function for 1D table lookup interpolation, with table / axis defined in params.yaml
It first finds the neighboring axis values using getNeighborsIx above. Then it determines the normalized relative position of the input value,
to it's neighboring elements, and does a linear interpolation to determine the right table value for the input value.
* servoCmdFromLinScale: This function is intended to be used to convert the normalized steering input [-1..1] into a servo command for the
pwm servo driver. This is done using experimentally determined parameters representing "Full Left", "Center", and "Full Right" steering positions.
A linear interpolation is used then to determine the servo command fot any steering input. Note: left and right use differen't interpolations / lookup
tables. This was done because left & right were quite different on my truck.
* main:
** read in parameters to global vars, and kill ROS if cant find param
** setup input topics
** setup output topics
** determine servo scaling using parameter axis / table
** main loop:
*** determine control condition, in this order of decending priority: 
**** no human detected, command safe
**** teleop steering detected, command using linear scaling of full left / center / full right pwm command
**** autonomous steering detected, command using conversion from turn radius to pwm command
*** build ros message & publish
