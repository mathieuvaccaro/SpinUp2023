/*
* Ctrl+k Ctrl+s : opens a fennetre with a tower of shortcuts 
* Ctrl+Shift+U : Upload
* Ctrl+Shift+B : Build
* Ctrl+Shift+M : Buil & Uplod 
*/

#include "main.h"
#include "okapi/api.hpp"


using namespace okapi;


// Sensors and Actuators ports
#define PORT_FL_WHEEL 2
#define PORT_FR_WHEEL 10
#define PORT_BL_WHEEL 11
#define PORT_BR_WHEEL 19
#define PORT_GYROSCOPE 20
#define PORT_PNEUMATICS 'A'
#define PORT_R_FLYWHEEL 9
#define PORT_L_FLYWHEEL 1

// Wheels specifications
#define WHEEL_DIRECTION_FL false
#define WHEEL_DIRECTION_FR true
#define WHEEL_DIRECTION_BL true
#define WHEEL_DIRECTION_BR false
#define WHEEL_GEARSET AbstractMotor::gearset::green
#define WHEEL_ENCODER_UNIT AbstractMotor::encoderUnits::rotations

// Flywheel specifications
#define FLYWHEEL_GEARSET AbstractMotor::gearset::green
#define FLYWHEEL_ENCODER_UNIT AbstractMotor::encoderUnits::degrees
#define FLYWHEEL_DIRECTION_R false
#define FLYWHEEL_DIRECTION_L true
#define FLYWHEEL_MAX_VELOCITY 200

// Proportions
#define WHEEL_DIAMETER 16_cm
#define WHEEL_TRACK 43_cm

// Global variables - sensors and actuators

Controller controller;
Motor motorFL = Motor(PORT_FL_WHEEL, WHEEL_DIRECTION_FL, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor motorFR = Motor(PORT_FR_WHEEL, WHEEL_DIRECTION_FR, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor motorBL = Motor(PORT_BL_WHEEL, WHEEL_DIRECTION_BL, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor motorBR = Motor(PORT_BR_WHEEL, WHEEL_DIRECTION_BR, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
pros::ADIPort pneumatic = pros::ADIPort(PORT_PNEUMATICS, ADI_DIGITAL_OUT);
IMU gyroscope = IMU(PORT_GYROSCOPE,okapi::IMUAxes::z);
Motor motorFlywheelLeft = Motor(PORT_L_FLYWHEEL, FLYWHEEL_DIRECTION_L, FLYWHEEL_GEARSET, FLYWHEEL_ENCODER_UNIT);
Motor motorFlywheelRight = Motor(PORT_R_FLYWHEEL, FLYWHEEL_DIRECTION_R, FLYWHEEL_GEARSET, FLYWHEEL_ENCODER_UNIT);
std::shared_ptr<ChassisController> drive;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	const MotorGroup leftMotors = {motorBL, motorFL};
	const MotorGroup rightMotors = {motorBR, motorFR};
	drive = ChassisControllerBuilder()
				.withMotors(leftMotors, rightMotors)
				.withDimensions(WHEEL_GEARSET, {{WHEEL_DIAMETER, WHEEL_TRACK}, imev5GreenTPR})
				.build();


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

void Shoot()
{
		const unsigned int delay = 3500 ; // Delay between wheel lift and push button pressure in ms (Default: 3500ms)
		motorFlywheelRight.moveVelocity(FLYWHEEL_MAX_VELOCITY);
		motorFlywheelLeft.moveVelocity(FLYWHEEL_MAX_VELOCITY/2);
		pros::delay(delay);
		pneumatic.set_value(1);
		pros::delay(1000);
		pneumatic.set_value(0);
		motorFlywheelRight.moveVelocity(0);
		motorFlywheelLeft.moveVelocity(0);
}

void autonomous() {
	
	const unsigned int delay = 0; // Delay entre chaque instruction en ms (Default : 0ms)
	const std::string delimiter = " "; //Delimiter between the instruction and the value (Default : ' ')

	// this is an array of strings that contains the instructions and thier values
	// Attention, put spaces only between the instructions and the values
	std::string instructions[] = {
		"Translation 3", 
		"Translation -1", 
		"Rotation 90", 
		"Rotation -90", 
		"Tirer"
		};

	for (size_t i = 0; i < instructions->size(); i++)
	{
		std::string instruction = instructions[i];
		std::string token = instruction.substr(0, instruction.find(delimiter)); // Instruction (Translation, Rotation, Tirer)
		std::string value = instruction.substr(instruction.find(delimiter) + 1, instruction.size()); // Value of the instruction
		pros::lcd::set_text(0, "Etape N" + i.toString());

		if (token == "Translation")
		{
			pros::lcd::set_text(1, "Translation de " + value + " m en cours...");	
			drive->moveDistance(stod(value) * meter);
		}
		else if (token == "Rotation")
		{
			pros::lcd::set_text(1, "Rotation de " + value + " m en cours...");
			drive->turnAngle(stod(value) * degree);
		}
		else if (token == "Tirer")
		{
			pros::lcd::set_text(1, "Tir en cours...");
			Shoot();
		}
		pros::lcd::set_text(1, "Instruction terminée !");
		pros::delay(delay);
	}
	

}

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

void opcontrol()
{
	bool execute = true;
	bool pneumaticActivated = false;
	bool debounceY = false;
	bool debounceA = false;
	bool debounceR1 = false;
	bool debounceR2 = false;
	bool flyWheelActivated = false;
	bool trigger = false;
	bool semiAuto = false;
	while (execute)
	{
		if (controller.getDigital(ControllerDigital::R1))
		{
			if (!debounceR1)
			{
				trigger = !trigger;
				debounceR1 = true;
			}
		}
		else
		{
			debounceR1 = false;
		}

		if (controller.getDigital(ControllerDigital::R2))
		{
			if (!debounceR2)
			{
				semiAuto = !semiAuto;
				debounceR2 = true;
			}
		}
		else
		{
			debounceR2 = false;
		}

		if (controller.getDigital(ControllerDigital::X))
		{
			execute = false;
		}

		if (controller.getDigital(ControllerDigital::A))
		{
			if (!debounceA)
			{
				flyWheelActivated = !flyWheelActivated;
				debounceA = true;
			}
		}
		else
		{
			debounceA = false;
		}

		if (trigger)
		{
			pneumatic.set_value(1);
			pros::delay(1000);
			pneumatic.set_value(0);
			trigger = !trigger;
		}
		else
		{
			pneumatic.set_value(0);
		}

		if (flyWheelActivated)
		{
			motorFlywheelRight.moveVelocity(FLYWHEEL_MAX_VELOCITY);
			motorFlywheelLeft.moveVelocity(FLYWHEEL_MAX_VELOCITY/2);

		}
		else
		{
			motorFlywheelRight.moveVelocity(0);
			motorFlywheelLeft.moveVelocity(0);
		}
		if (controller.getAnalog(ControllerAnalog::rightY)==-1)
		{
			drive->getModel()->arcade(
				((controller.getAnalog(ControllerAnalog::leftY))/4),
				(controller.getAnalog(ControllerAnalog::leftX))/4);
		}
		else
		{
			drive->getModel()->arcade(
				((controller.getAnalog(ControllerAnalog::leftY))),
				(controller.getAnalog(ControllerAnalog::leftX)));
		}
		
		pros::delay(100);
	}
}
