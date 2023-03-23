/*
* Ctrl+k Ctrl+s : opens a fennetre with a tower of shortcuts 
* Ctrl+Shift+U : Upload
* Ctrl+Shift+B : Build
* Ctrl+Shift+M : Buil & Uplod 
*/

#include "main.h"
#include "okapi/api.hpp"

#include <string>

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
#define PORT_ELEVATOR_BOTTOM 4
#define PORT_ELEVATOR_TOP 8
#define PORT_DOOR_DISC 5
#define PORT_LEFT_CATAPULTE 13
#define PORT_RIGHT_CATAPULTE 12

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
#define FLYWHEEL_MAX_VELOCITY 250

// ELEVATOR specifications 
#define ELEVATOR_GEARSET AbstractMotor::gearset::green
#define ELEVATOR_ENCODER_UNIT AbstractMotor::encoderUnits::degrees
#define DOOR_DISC false
#define ELEVATOR_BOTTOM false
#define ELEVATOR_TOP false
#define DOOR_MAX_VELOCITY 100
#define ELEVATOR_MAX_VELOCITY 220

//CATAPULTE
#define CATAPULTE_DIRECTION_R true
#define CATAPULTE_DIRECTION_L false
#define CATAPULTE_GEAREST AbstractMotor::gearset::green
#define CATAPULTE_ENCODER_UNIT AbstractMotor::encoderUnits::degrees
#define CATAPULTE_MAX_VELOCITY 200

//GPS
#define distanceGPS 0 // en cm

// Proportions
#define WHEEL_DIAMETER 16_cm
#define WHEEL_TRACK 43_cm

// Global variables - sensors and actuators

Controller controller;
Motor motorFL = Motor(PORT_FL_WHEEL, WHEEL_DIRECTION_FL, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor motorFR = Motor(PORT_FR_WHEEL, WHEEL_DIRECTION_FR, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor motorBL = Motor(PORT_BL_WHEEL, WHEEL_DIRECTION_BL, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor motorBR = Motor(PORT_BR_WHEEL, WHEEL_DIRECTION_BR, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor CatapulteL = Motor(PORT_LEFT_CATAPULTE, CATAPULTE_DIRECTION_L, CATAPULTE_GEAREST, CATAPULTE_ENCODER_UNIT);
Motor CatapulteR = Motor(PORT_RIGHT_CATAPULTE, CATAPULTE_DIRECTION_R, CATAPULTE_GEAREST, CATAPULTE_ENCODER_UNIT);
pros::ADIPort pneumatic = pros::ADIPort(PORT_PNEUMATICS, ADI_DIGITAL_OUT);
IMU gyroscope = IMU(PORT_GYROSCOPE,okapi::IMUAxes::z);
Motor motorFlywheelLeft = Motor(PORT_L_FLYWHEEL, FLYWHEEL_DIRECTION_L, FLYWHEEL_GEARSET, FLYWHEEL_ENCODER_UNIT);
Motor motorFlywheelRight = Motor(PORT_R_FLYWHEEL, FLYWHEEL_DIRECTION_R, FLYWHEEL_GEARSET, FLYWHEEL_ENCODER_UNIT);
Motor motorDOOR = Motor(PORT_DOOR_DISC, DOOR_DISC,ELEVATOR_GEARSET, ELEVATOR_ENCODER_UNIT);
Motor motorElevatorB = Motor(PORT_ELEVATOR_BOTTOM, ELEVATOR_BOTTOM,ELEVATOR_GEARSET, ELEVATOR_ENCODER_UNIT);
Motor motorElevatorT = Motor(PORT_ELEVATOR_TOP, ELEVATOR_TOP,ELEVATOR_GEARSET, ELEVATOR_ENCODER_UNIT);
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
   
    const unsigned int delay = 0; // Delay between each instruction in ms (Default : 0ms)
    const std::string delimiter = " "; //Delimiter between the instruction and the value (Default : ' ')
    const unsigned int SquareSize = 60;

    // this is an array of strings that contains the instructions and thier values
    // Attention, put spaces only between the instructions and the values
    const std::vector instructions = std::vector({
        "Translation 1",
        "Translation -1",
        "Rotation 90",
        "Rotation -90",
        "Translation 0.5",  
    });

    for (int i = 0; i < instructions; i++)
    {
        std::string instruction = instructions[i];
        std::string token = instruction.substr(0, instruction.find(delimiter)); // Instruction (Translation, Rotation, Tirer)
        std::string value = instruction.substr(instruction.find(delimiter) + 1, instruction.size()); // Value of the instruction

        if (token == "Translation")
        {  
            drive->moveDistance(stod(value) * SquareSize * centimeter);
        }
        else if (token == "Rotation")
        {
            drive->turnAngle(stod(value) * degree);
        }
        else if (token == "Tirer")
        {
            Shoot();
        }
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
    bool debounceB = false;
    bool debounceR1 = false;
    bool debounceR2 = false;
    bool flyWheelActivated = false;
    bool doorActivated = false;
    bool elevatorBActivated = false;
    bool elevatorTActivated = false;
    bool catapulteLeftActivated = false;
    bool catapulteRightActivated = false;
    bool trigger = false;
    bool semiAuto = false;

    // GPS
    int GPSCoordX = 0;
    int GPSCoordY = 0;
    int GPSAngle = 0;
   
    pros::lcd::initialize;

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

        // End game
        if (controller.getDigital(ControllerDigital::X))
        {
            CatapulteL.moveVelocity(CATAPULTE_MAX_VELOCITY);
            CatapulteR.moveVelocity(CATAPULTE_MAX_VELOCITY);
            pros::delay(100);
            CatapulteL.moveVelocity(0);
            CatapulteR.moveVelocity(0);
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

        if (controller.getDigital(ControllerDigital::B))
        {
            if (!debounceB)
            {
                doorActivated = !doorActivated;
                elevatorBActivated = !elevatorBActivated;
                elevatorTActivated = !elevatorTActivated;
                debounceB = true;
            }
        }
        else
        {
            debounceB = false;
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
        if (doorActivated)
        {
            motorDOOR.moveVelocity(DOOR_MAX_VELOCITY);
        }
        else
        {
            motorDOOR.moveVelocity(0);
        }
        if (elevatorBActivated)
        {
            motorElevatorB.moveVelocity(ELEVATOR_MAX_VELOCITY);
        }
        else
        {
            motorElevatorB.moveVelocity(0);
        }
        if (elevatorTActivated)
        {
            motorElevatorT.moveVelocity(ELEVATOR_MAX_VELOCITY);
        }
        else
        {
            motorElevatorT.moveVelocity(0);
        }
        if (flyWheelActivated)
        {
            motorFlywheelRight.moveVelocity(FLYWHEEL_MAX_VELOCITY);
            motorFlywheelLeft.moveVelocity(FLYWHEEL_MAX_VELOCITY);

        }
        else
        {
            motorFlywheelRight.moveVelocity(0);
            motorFlywheelLeft.moveVelocity(0);
        }
        if (controller.getAnalog(ControllerAnalog::rightY)==-1)
        {
            if(controller.getDigital(ControllerDigital::L1))
            {
            drive->getModel()->arcade(
                ((controller.getAnalog(ControllerAnalog::leftY))/4*-1),
                (controller.getAnalog(ControllerAnalog::leftX))/4*-1);
            GPSCoordX += (controller.getAnalog(ControllerAnalog::leftX)/4*-1);
            GPSCoordY += (controller.getAnalog(ControllerAnalog::leftY)/4*-1);
            }
            else
            {
            drive->getModel()->arcade(
                ((controller.getAnalog(ControllerAnalog::leftY))/4),
                (controller.getAnalog(ControllerAnalog::leftX))/4);
            GPSCoordX += (controller.getAnalog(ControllerAnalog::leftX)/4);
            GPSCoordY += (controller.getAnalog(ControllerAnalog::leftY)/4);
            }
        }
        else
        {
            if(controller.getDigital(ControllerDigital::L1))
            {
            drive->getModel()->arcade(
                ((controller.getAnalog(ControllerAnalog::leftY)*-1)),
                (controller.getAnalog(ControllerAnalog::leftX)*-1));
            GPSCoordX += (controller.getAnalog(ControllerAnalog::leftX)*-1);
            GPSCoordY += (controller.getAnalog(ControllerAnalog::leftY)*-1);
            }
            else
            {
            drive->getModel()->arcade(
                ((controller.getAnalog(ControllerAnalog::leftY))),
                (controller.getAnalog(ControllerAnalog::leftX)));
            GPSCoordX += (controller.getAnalog(ControllerAnalog::leftX));
            GPSCoordY += (controller.getAnalog(ControllerAnalog::leftY));
            }
        }
       
        pros::delay(100);
    }
}