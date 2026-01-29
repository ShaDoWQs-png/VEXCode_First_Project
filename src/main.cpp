#include "main.h"
#include "./lemlib/api.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({8, 9, 10});  // Left motor on port 1
pros::MotorGroup right_mg({1, 2, 3}); // Right motor on port 2, reversed
pros::Imu inertial(4);
pros::Rotation yEnc(6);  // Rotation sensor on port 1
pros::Rotation xEnc(7);  // Rotation sensor on port 2

lemlib::TrackingWheel vertWheel(&yEnc, lemlib::Omniwheel::NEW_275, -5);    // Vertical tracking wheel with 2.75" diameter, 5 inches left of center
lemlib::TrackingWheel horizWheel(&xEnc, lemlib::Omniwheel::NEW_275, -2);  // Horizontal tracking wheel with 2.75" diameter, 2 inches behind center

lemlib::OdomSensors odomSensors(&vertWheel, nullptr, &horizWheel, nullptr, &inertial);
lemlib::Drivetrain drivetrain(&left_mg, &right_mg, 12, lemlib::Omniwheel::NEW_275, 200, 2);
lemlib::ControllerSettings linContrSettings(
		10, // Proportional
        0, // Integral
        3, // Derivative

        3, // Integral anti windup range
        1, 	// Small error range (in.)
        100, // Small error range timeout (ms)
        3, // Large error range (in.)
        500, // Large error range timeout (ms)
        5 // Maximum acceleration (slew)
);
lemlib::ControllerSettings angContrSettings(
		10,
        0,
        3,

        3,
        1,
        100,
        3,
        500,
        5
);

lemlib::Chassis chassis(
	drivetrain,
	linContrSettings,
	angContrSettings,
	odomSensors,
	nullptr
);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

	inertial.reset();
	while(inertial.is_calibrating()) {
		pros::delay(10);
	}

	chassis.calibrate();
	chassis.setPose(0, 0, 0);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int turn = controller.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int dir = controller.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage
		std::cout << "X: " << chassis.getPose().x << " Y: " << chassis.getPose().y << " A: " << chassis.getPose().theta << std::endl;
		pros::delay(20);                               // Run for 20 ms then update
	}
}