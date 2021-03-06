#include "main.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "drive.h"
#include "all_used.h"
#include "lift.h"
#include "angler.h"
#include "intake.h"

const int LIFT_HIGH = 2500;
const int LIFT_LOW = 1800;
const int LIFT_DESCORE = 1700;

void opcontrol() {
	// global variables
	int stickArray[4]; // temporary power until limited
	int power[4]; // power to the array
	bool intakeUsed = false;
	bool driveBackToggle = false;
	bool anglerVal = false; // manage angler's states on one button
	bool liftBool = false; // used to check first lift

	while (true) {
		float armPosition = arm.get_position();
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			stickArray[0] = 127;
			stickArray[2] = 127;
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			stickArray[0] = -127;
			stickArray[2] = -127;
		} else {
			stickArray[0] = 0;
			stickArray[2] = 0;
		}
		// stickArray[0] = powf(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), 3) / powf(127, 2);
		stickArray[1] = powf(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 3) / powf(127, 2);
		// stickArray[2] = powf(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 3) / powf(127, 2);
		stickArray[3] = powf(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y), 3) / powf(127, 2);
		// loops through array and removes values under 15 from the calculation
		for (size_t j = 0; j < 4; j++) {
			if (abs(stickArray[j]) < 15) {
				stickArray[j] = 0;
			}
			// for x values remove if they are under 30
			if (j == 0 || j == 2) {
				if (127 - abs(stickArray[j]) < 30) {
					if (stickArray[j] > 0)
						stickArray[j] = 127;
					else
						stickArray[j] = -127;
				}
			}
		}
		// tank drive with mecanum calculations
		power[0] = stickArray[1] + stickArray[0];
		power[1] = stickArray[1] - stickArray[0];
		power[2] = stickArray[3] - stickArray[2];
		power[3] = stickArray[3] + stickArray[2];
		// loops through all power values to check if they are above the limit (127)
		for (size_t i = 0; i < 4; i++) {
			if (abs(power[i]) > 127) {
				if (power[i] > 0) {
					power[i] = 127;
				} else {
					power[i] = -127;
				}
			}
		}
		// enabling the drive back stack toggle
		if (!driveBackToggle) {
			// sets drive to power
			drive_left = power[0];
			drive_left_b = power[1];
			drive_right = power[2];
			drive_right_b = power[3];
		} else {
			std::uint32_t now = pros::millis();
			loader_left.move_voltage(-12000);
			loader_right.move_voltage(-12000);
			angler_pid(0, true, 127, false, 2000);
			drive_set(-90);
			pros::Task::delay_until(&now, 1000);
			driveBackToggle = false;
		}

		// check if intake is used in any task, letting driver use it.
		if (intakeTaskBool || !anglerIntakeThreshold || autoIntakeBool || sensorOutakeBool || driveBackToggle) {
			intakeUsed = true;
		} else {
			intakeUsed = false;
		}
		// intake on triggers
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && !intakeUsed) {
			loader_left.move_voltage(12000);
			loader_right.move_voltage(12000);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && !intakeUsed) {
			loader_left.move_velocity(-12000);
			loader_right.move_velocity(-12000);
		} else if (!intakeUsed) {
			loader_left.move(0);
			loader_right.move(0);
		}

		// autonomous stacking mechanism
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			// go foward
			if (!anglerVal) {
				anglerHold = false;
				pros::delay(20);
				angler_pid(-4500, true, 127, false);
			} else if (anglerVal) { // go backward
				anglerHold = false;
				pros::delay(20);
				angler_pid(0, true, 127, false, 2000);
			}
			// same button to return
			anglerVal ? anglerVal = false : anglerVal = true;
		}

		// toggle to drive back
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
			driveBackToggle ? driveBackToggle = false : driveBackToggle = true;
		}

		// lift high scoring value
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			if (!(LIFT_HIGH + 100 > armPosition && armPosition > LIFT_HIGH - 100)) {
				lift(LIFT_HIGH, 20000);

				if (!liftBool) {
					liftBool = true;
					sensor_outtake();
				}
			}
		}

		// lift descore value
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			if (!(LIFT_LOW + 100 > armPosition && armPosition > LIFT_LOW - 100)) {
				lift(LIFT_LOW, 20000);

				if (!liftBool) {
					liftBool = true;
					sensor_outtake();
				}
			}
		}

		// drop lift
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
			lift(0, 1000);
			liftBool = false;
		}

		// reset stuff
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			angler.move(80);
			angler.tare_position();
			lift(0, 20);
			liftBool = false;
			intakeTaskBool = false;
			anglerIntakeThreshold = true;
		} else if (!anglerBool) {
			angler.move(0);
		}

		pros::delay(20);
	}

}
