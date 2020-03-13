# 4549A
> VEX Robotics Code For the Envertronics Team: 4549A during the Tower Takeover season.

 #### Table of Contents
 * [The Basics](#The-Basics)
 * [Opcontrol](#Opcontrol)
 * [PID](#PID)
 * [Angler PID Task](#angler-pid)
 * [Lift PID Task](#Lift-PID-Task)
 * [Light Sensor](#Light-Sensor)
 * [Vision Sensor Cube Tracking](#Vision-Sensor-Cube-Tracking)
 * [Intakes](#Intakes)

### Programming And Algorithm Design Proccess
> For an in-depth look at the programming process, [click here](https://github.com/Sajantoor/4549A-Code/blob/master/docs/extra-docs.md).

## The Basics
### Initialize
 > The initialize file is used to define tasks for future use.

An example of a task definition:
 ```cpp
 pros::task_t tracking_task = pros::c::task_create(tracking_update, (void*)NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "TRACKING TASK");
 ```

[View Initialize](../master/src/initialize.cpp)

### Motors and Sensors
> Motors and sensors are defined in their own file.

 An example of a motor definition:
 ```cpp
 pros::Motor drive_left(DRIVE_LEFT, MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
 ```

### Motor Sensor Init
> This is very all the motors, sensors and ports for the motors are defined.
```cpp
#define DRIVE_LEFT 9
#define pot_port_arm 1
extern pros::Motor drive_left;
pros::Motor drive_left(DRIVE_LEFT, MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::ADIPort potentiometer_arm (pot_port_arm, pros::E_ADI_ANALOG_IN);
```

 [View motors and sensors](../master/src/motor_setup.cpp)

 ## Opcontrol
 > This is where all the driver related code is. It's designed to be as simple as possible for the driver and requires as few inputs as possible from the driver allowing them to focus on more important aspects of the game.

 ```cpp
// autonomous stacking mechanism
if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
	// go foward
      if (!anglerVal) {
	 anglerHold = false;
	  pros::delay(20);
	  angler_pid(-4500, true, 127, true);
      } else if (anglerVal) { // go backward
          anglerHold = false;
         pros::delay(20);
         angler_pid(0, true, 127, false, 2000);
     }
  // same button to return
  anglerVal ? anglerVal = false : anglerVal = true;
}
 ```
> This code allows the driver to, at a button press, stack cubes with the angler and the angler returns to its original position. This is a task which allows the driver to do other things such as drive or intake, if they desired.

[View opcontrol](../master/src/opcontrol.cpp)


 ## PID
 > This is our non redundant, PID function. PID values such as kp, kd, etc. are entered using a constructor inside a struct. These values are used with a target and sensor values to calculate what power the motor should be at that given time.

 ```cpp
typedef struct pid_values {
  float Kp, Kd, Ki, integral_limit, error, last_error, power, integral_active_zone, calc_power;
  float integral, derivative, proportional, target;
  int max_power;
  // constructor
  pid_values(float Kp, float Kd, float Ki, float integral_limit, float integral_active_zone, int max_power):
    Kp(Kp), Kd(Kd), Ki(Ki), integral_limit(integral_limit), integral_active_zone(integral_active_zone),
    max_power(max_power), error(0), last_error(0), power(0), calc_power(0) {
  }
} pid_values;
```
[View the struct](../master/include/pid.h)


> This is the struct with its constructor. It's designed to be as flexible as possible and non redundant. Inside our function we can manipulate different aspects of this struct such as the derivative value if we so desired using the "dot (.)" operator. `<constructor id>.derivative`


 ```cpp
 pid_values angler_pid(0.31, 0.07, 0, 30, 500, 70);
 ```

This piece of code creates a struct with the id of "angler_pid" using the pid_values constructor (above). These arguments are passed according to the constructor parameters (kp is the first argument within the constuctor, therefore 0.31 will be the kp).


```cpp
float pid_calc(pid_values *pid, float target, float sensor) {
   pid->target = target;
   pid->error = pid->target - sensor;
   float derivative = (pid->error - pid->last_error);
   pid->last_error = pid->error;
   float proportional = pid->error;
   float integral = pid->error + integral;

   if (integral*pid->Ki > pid->integral_limit) {
     integral = pid->integral_limit;
   }

   if (integral*pid->Ki < -pid->integral_limit) {
     integral = -pid->integral_limit;
   }

   if (fabs(pid->error) > pid->integral_active_zone) {
     integral = 0;
   }

   pid->calc_power = (proportional*pid->Kp) + (integral*pid->Ki) + (derivative*pid->Kd);
   // calculates power then returns power as max power
   pid->power = power_limit(pid->max_power, pid->calc_power);
   return pid->power;
}
```


This function runs the PID calculations using pointers. It takes in the arguments of the struct (which is called "pid" because of the pointer), target and the sensor we are using, sometimes this is encoders, other times it's potentiometer values. The motor is given the return value of `pid.power`, which is the final power calculated.

This function is called like this:
```cpp
float final_power = pid_calc(&lift_pid, height, position);
motor.move(final_power);
```

One of the functions called in the "pid_calc" function is "power_limit". It performs a simple check and returns with a float value. It takes in 2 arguments, "allowed_speed" (max speed) and "actual_speed". It checks whether or not the calculated PID speed is greater than the max speed allowed and returns a value accordingly.

```cpp
float power_limit(float allowed_speed, float actual_speed) {
   if (actual_speed > allowed_speed) {
       actual_speed = allowed_speed;
   }

   else if (actual_speed < -allowed_speed) {
       actual_speed = -allowed_speed;
   }

   return actual_speed;
}
```

[View PID](../master/src/pid.cpp)

## Angler PID Task
> The angler task allows for autonomous and accurate stacking in both the autonomous and driver control periods. The angler task uses the PID calculations and motor encoders to move to certain positions. Using torque values from the motor we are able to calculate the number of cubes in the tray and move the tray accordingly. Before stacking, the light sensor is used to outake to the perfect position allowing for an increased degree of accuracy.

```cpp
void angler_pid_task(void*ignore) {
    ...
    while (anglerBool) {
      if (anglerIntakeThreshold || (currentTarget == TRAY_BACKWARD_VAL)) {
        // angler stack code
        anglerIntakeThreshold = true;
        if (anglerDelay && (pros::millis() > timeout)) {
          delayReached = true;
        }

        // max torque value is used to calculate how many cubes are in the angler
        if (pros::c::motor_get_torque(ANGLER) > maxTorque) {
          maxTorque = pros::c::motor_get_torque(ANGLER);
        }

        // 8 stack torque is faster than 7 stack
        if (maxTorque > EIGHT_STACK_TORQUE && (fabs(angler_pid.error) < 1400)) {
          if (angler_pid.max_power < currentSpeed * 0.5) {
            angler_pid.max_power = currentSpeed * 0.5;
          } else {
            angler_pid.max_power = angler_pid.max_power - 15;
          }
        // 7 stack torque is slower
        } else if (maxTorque > SEVEN_STACK_TORQUE && (fabs(angler_pid.error) < 1200)) {
          if (angler_pid.max_power < currentSpeed * 0.4) {
            angler_pid.max_power = currentSpeed * 0.4;
          } else {
            angler_pid.max_power = angler_pid.max_power - 25;
          }
        // slow down for all cubes
        } else {
          if (fabs(angler_pid.error) < 1000) {
            if (angler_pid.max_power < currentSpeed * 0.5) {
              angler_pid.max_power = currentSpeed * 0.5;
            } else {
              angler_pid.max_power = angler_pid.max_power - 15;
            }
          } else {
            angler_pid.max_power = currentSpeed;
          }
        }

        float currentTime = pros::millis();
        float position = angler.get_position();
        int final_power = pid_calc(&angler_pid, currentTarget, position);
        angler.move(final_power);
        // exits out of the loop after the error has been reached, hold value has been reached
        if ((fabs(angler_pid.error) <= ERROR_THRESHOLD) || !anglerHold || delayReached || (ignoreError && nextTarget)) {
	  ...
        }
      } else {
        // outtake cube to be at the bottom of the tray
        if (light_sensor_intake.get_value() < 1850) { // if cube is detected
          loader_left.move(0);
          loader_right.move(0);
          anglerIntakeThreshold = true;
        } else if (light_sensor_intake.get_value() > 1850) { // if cube isn't detected
          if (pros::millis() > intakeThresholdTimer) {
	    ....
          } else {
            loader_left.move(-70);
            loader_right.move(-70);
          }
        }
      }

      pros::delay(20);
    }
    pros::delay(20);
  }
}

 ```

[View Angler](../master/src/angler.cpp)

## Lift PID Task
> The Lift PID task uses the same PID calculations as the angler. However it's simpler as we don't have to worry about torque.

```cpp
  while(liftBool) {
      // calculates time for timeout
      if (timer) {
        failsafe = pros::millis() + timeout + hold;
        delayTime = pros::millis() + hold;
        timer = false;
      }

       float currentTime = pros::millis();
       float position = arm.get_position(); // mtr encoders
       float final_power = pid_calc(&lift_pid, height, position); // final power is calculated using pid
       arm.move(final_power);
       // slew rate to slow down the motor based on the error value
       if (position > 1980) {
         lift_pid.max_power = lift_pid.max_power - 5; // slew rate
         if (lift_pid.max_power < 60) lift_pid.max_power = 60; // capping lowest possible speed
       } else {
         lift_pid.max_power = lift_pid.max_power + 25; // positive slew rate
         if (lift_pid.max_power > 127) lift_pid.max_power = 127; // motor cap
       }

       // exit out of the loop
       if ((fabs(lift_pid.error) < 10) && (currentTime > delayTime)) {
         lift_pid.max_power = 127;
         hold = 0;
         arm.move(0);
         liftBool = false;
       // exit out based off the failsafe
       } else if (failsafe < currentTime) {
         lift_pid.max_power = 127;
         hold = 0;
         arm.move(0);
         liftBool = false;
       }
     }
     pros::delay(20);
  }
}
```

[View Lift](../master/src/lift.cpp)

## Light Sensor
> The light sensor is used in several parts of the code, from the autonomous intake function, which combines the light sensor and vision sensor readings to intake only when a cube is close to the robot, outaking precisely for cube lock using PID loops, and outaking for a precise stack.  


## Vision Sensor Cube Tracking
> Vision sensor is used to recognize, track and follow cubes. It uses 3 signatures, one for each cube colour and then uses drive functions to follow the cubes. It allows us to dynamically make adjustments to our autonomous based on field conditions, an example of this would be cubes are setup with a margin of error or for cubes falling unpredictably. This allows for us to maximize the number of cubes we get during the autonomous period.

```cpp
struct data {
  int width; // effective width of the cube
  int height; // effective height of the cube
  int size; // area, width * height
  int x; // x value
  int y; // y value
  int id; // id for the cube color
  bool deepVisionCheck; // see if the deep vision function is used
};
```

> This code is used for to manage data for the cube using a struct, this data is manipulated in the following functions.

```cpp
int targetSelection() {
  int detectionValues[3]; // y value of all detection colors
  int closestCube = 0;
  // detecting values
  pros::vision_object_s_t purpleDetection = vision_sensor.get_by_sig(0, PURPLE);
  pros::vision_object_s_t orangeDetection = vision_sensor.get_by_sig(0, ORANGE);
  pros::vision_object_s_t greenDetection = vision_sensor.get_by_sig(0, GREEN);

  // check size of all detections
  detectionValues[0] = greenDetection.y_middle_coord;
  detectionValues[1] = orangeDetection.y_middle_coord;
  detectionValues[2] = purpleDetection.y_middle_coord;
  // gets the closest cube from the array and the color id
  for (size_t i = 0; i < 3; i++) {
    if ((vision_sensor.get_object_count() > 1) && (detectionValues[i] > DEEP_VISION_THRESHOLD)) {
      detectionValues[i] = deepVision(i + 1);
    }

    if (detectionValues[i] > closestCube) {
      closestCube = detectionValues[i];
      cubeColor = i + 1;
    }
  }
  // if the cube is bigger than smallest possible cube, target is selected
  if (closestCube > DETECTION_THRESHOLD) {
    if (closestCube == deepVisionData.y) {
      currentCube = deepVisionData;
    } else {
      clearData(&deepVisionData);
    }
  } else {
    closestCube = 0;
    cubeColor = 0;
    return 0;
  }

  return closestCube;
}
```
> This code selects a target cube colour to be the closest cube from the for loop and that cubes's coordinates are used in the motion and tracking part of the code. It also has to meet specfic thresholds to see if the cube is close enough to track for example `DETECTION_THRESHOLD`

### Deep Vision Algorithm
```cpp
int deepVision(int id) {
  int numObjects = vision_sensor.get_object_count();
  float smallestLeft; // smallest left coordinate
  float largestLeft; // largest
  float largestLeftWidth; // width of largest to calculate the right coordinate
  float largestTop; // largest top coordinate
  float smallestTop; //smallest
  float smallestTopHeight; // height of smallest top to calculate bottom coordinate

  if (numObjects > 3) numObjects = 3;

  pros::vision_object_s_t partArray[numObjects];

  for (size_t i = 0; i < numObjects; i++) {
    partArray[i] = vision_sensor.get_by_sig(i, id);

    if (partArray[i].width * partArray[i].height < MIN_DEEP_VISION_THRESHOLD) {
      // printf("Deep vision cancelled \n \n");
      break;
    }

    if (partArray[i].left_coord > largestLeft) {
      largestLeft = partArray[i].left_coord;
      largestLeftWidth = partArray[i].width;
    }
    // not else if here because what if smallest left is the first value, will cause bugs
    if (partArray[i].left_coord < smallestLeft) {
      smallestLeft = partArray[i].left_coord;
    }

    if (partArray[i].top_coord > largestTop) {
      largestTop = partArray[i].top_coord;
    }

    if (partArray[i].top_coord < smallestTop) {
      smallestTop = partArray[i].top_coord;
      smallestTopHeight = partArray[i].height;
    }
  }

  deepVisionData.deepVisionCheck = true;
  deepVisionData.width = (largestLeft + largestLeftWidth) - smallestLeft;
  deepVisionData.height = largestTop - (smallestTop - smallestTopHeight);
  deepVisionData.size = deepVisionData.width * deepVisionData.height;
  deepVisionData.x = smallestLeft + (deepVisionData.width / 2);
  deepVisionData.y = largestTop - (deepVisionData.height /2);

  if (deepVisionData.size < 0) {
    return 0;
  }

  return deepVisionData.y;
}
```
> Due to low lighting conditions the vision sensor may not detect the whole cube, there may be multiple smaller detections. Then this algorithm steps in. This algorithm applies basic geometric operations to calculate the full width of the cube based on these smaller detected pieces. This improves the performance of cube tracking in these low lighting conditions.

[View vision sensor code](../master/src/vision.cpp)

### Driving With Vision Sensor
> The whole point of designing an algorithm that is able to track cubes is to be able to correct for misaligned cubes during the autonomous period. Within the general purpose drive function, the vision sensor's cube tracking can be used after passing a parameter.

```cpp
if (max_error && !vision || max_error && currentCube.size < CUBE_SIZE_THRESHOLD_MIN) {
 // drive correction code
} else if (vision) {
  if (currentCube.size > CUBE_SIZE_THRESHOLD_MIN) {
    if (currentCube.x > CENTER_X) {
      cubeCorrectionDirection = 1;
     } else {
       cubeCorrectionDirection = -1;
    }

    if (fabs(currentCube.x + -CENTER_X) > 80) {
       cubeCorrection = false;
       turn_set(40 * cubeCorrectionDirection);
     } else {
       cubeCorrection = true;
     }
   } else {
     cubeCorrection = true;
  }
}
```
> If the vision sensor detects a cube large enough to be tracked then the cube correction takes over, turning off the drive correction. It turns according to the x values of the cube and lines it self up with the cube. Then the drive correction comes back and corrects for the misalignment due to the cube tracking. This assuring the robot never goes off the path.

## Intakes
> The intakes have two major systems, the autonomous intake system combining the vision and light sensor to intake and the light sensor based outake.

### Light Sensor Based Outake
> The light sensor based outtake is critical part of the cube lock system in our robot. It allows the driver to press a button and the cube is automatically in the intakes in the bot, allowing cubes to be scored.

```cpp
void sensor_outtake() {
  double sensorValue = light_sensor_intake.get_value();
  if (sensorValue > LIGHT_SENSOR_THRESHOLD) {
    intakePIDFunc(-700, 127);
   }
}
```
> The code is fairly simple, if the cube is blocking the light sensor, it doesn't need to be outaked, otherwise there is a PID loop which outakes the cubes.

### Autonomous Intake System
> The autonomous intake system is a task, meaning it can be run asynchronously on a separate thread on the V5 brain. This the autonomous intake system happen in parallel with something else. This system is used during the autonomous period.

```cpp
void autoIntake(void*ignore) {
  // time for intake to slow if intake doesn't detect cubes
  float lightSensorTimeout = 1500;
  float timer;
  bool intakeTimeout = false;

  while (true) {
    while (autoIntakeBool && pros::competition::is_autonomous()) { // task running bool
      double lightSensorValue = light_sensor_intake.get_value();
      if ((lightSensorValue < 1850) || currentCube.size > CUBE_SIZE_THRESHOLD_MAX) { // if cube is detected
        loader_left.move(127);
        loader_right.move(127);
        timer = lightSensorTimeout + pros::millis(); // timer is updated
      } else if (intakeTimeout) { // stop intakes
        loader_left.move(0);
        loader_right.move(0);
        intakeTimeout = false; // restart the timeout to have it run again if cube is detected
      } else if (timer < pros::millis()) { // run timer
        intakeTimeout = true;
      }

      pros::delay(20);
    }

    if (!pros::competition::is_autonomous()) {
      autoIntakeBool = false;
    }

    pros::delay(20);
  }
}
```
**How the system works**: Using the Vision Sensor we are able to track cubes, from that we are able to calculate the size of each cube. If the size is big enough, meaning it's close to the robot, the intakes run. The light sensor is also used, it's used to check if the cubes are fully intaked, if not the intakes run.

[View intake](../master/src/intake.cpp)
