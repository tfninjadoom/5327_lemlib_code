#include "main.h"
#include "lemlib/api.hpp"

// MACROS
#define DRIVE_GEARSET pros::E_MOTOR_GEARSET_06

// MOTOR PORTS
const int 
LEFT_DRIVE_PORTS[3] {-5, -4, 3};
const int 
RIGHT_DRIVE_PORTS[3] {1, 11, -6};

const int
INTAKE_PORTS[2] {20, 0};
const int 
SLAPPER_PORTS[1] {12};

// V5 SENSOR PORTS
const int
IMU_PORT {14};
const int 
VISION_PORT {0};
const int 
DISTANCE_PORT {0};
const int 
OPTICAL_PORT {0};
const int 
ROTATION_PORTS[3] {17, 18, 19};
const int 
GPS_PORT {0};

//----------------------------------------------------------------------------
// OBJECT INITIALIZATION

// Motors
pros::Motor         leftFront(LEFT_DRIVE_PORTS[0], DRIVE_GEARSET);
pros::Motor         leftBack(LEFT_DRIVE_PORTS[1], DRIVE_GEARSET);
pros::Motor         leftMid(LEFT_DRIVE_PORTS[2], DRIVE_GEARSET);


pros::Motor         rightFront(RIGHT_DRIVE_PORTS[0], DRIVE_GEARSET);
pros::Motor         rightBack(RIGHT_DRIVE_PORTS[1], DRIVE_GEARSET);
pros::Motor         rightMid(RIGHT_DRIVE_PORTS[2], DRIVE_GEARSET);

pros::Motor         intake(INTAKE_PORTS[0], pros::E_MOTOR_GEARSET_18);
pros::Motor         intake2(INTAKE_PORTS[1], pros::E_MOTOR_GEARSET_18);
pros::Motor         slapper(SLAPPER_PORTS[0], pros::E_MOTOR_GEARSET_36);

//Motor Groups
pros::MotorGroup    leftDrive  ( {leftFront, leftBack, leftMid} );
pros::MotorGroup    rightDrive ( {rightFront, rightBack, rightMid} );

// V5 Sensors
pros::Imu          imu(IMU_PORT);
pros::Vision        vision(VISION_PORT);
pros::Distance      distance(DISTANCE_PORT);
pros::Optical       optical(OPTICAL_PORT);
pros::Rotation      rotationP(ROTATION_PORTS[0]);
pros::Rotation      rotationI(ROTATION_PORTS[1]);
pros::Rotation      rotationD(ROTATION_PORTS[2]);
pros::GPS           gps(GPS_PORT);

// 3-Wire Sensors
// pros::ADIEncoder    encoder x4
// pros::ADIDigitalIn  bumper x2
// pros::ADIDigitalIn  limitSwitch x2
// pros::ADIAnalogIn   lineTracker x3


// Pneumatics
pros::ADIDigitalOut elevationWing('C');
pros::ADIDigitalOut leftWing('H');
pros::ADIDigitalOut rightWing('D');
//pros::ADIDigitalOut plowWings('B');


// tracking wheels
pros::Rotation horizontalEnc(7);
// horizontal tracking wheel. 2.75" diameter, 3.7" offset, back of the robot
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -3.7);

// drivetrain
lemlib::Drivetrain_t drivetrain {&leftDrive, &rightDrive, 15, lemlib::Omniwheel::NEW_275, 600, 40};

// lateral motion controller
lemlib::ChassisController_t lateralController {70, 210, 1, 100, 3, 500, 20};

// angular motion controller
lemlib::ChassisController_t angularController {2, 20, 1, 100, 3, 500, 20};

// sensors for odometry
lemlib::OdomSensors_t sensors {nullptr, nullptr, nullptr, nullptr, &imu};

lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();

    lemlib::Logger::initialize();

    // calibrate sensors
    chassis.calibrate();
    chassis.setPose(lemlib::Pose(0, 0, 0));

    // print odom values to the brain
    pros::Task screenTask([=]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            lemlib::Logger::logOdom(chassis.getPose());
            pros::delay(20);
        }
    });
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
void tuned_skills(){
    //Set the point of referance for future movements 
    chassis.setPose(-54, -50, -108);
    //matchload
    //open the right wing in order to knock the the match load triball closer to the goal
    pros::Task([=]{
        rightWing.set_value(1);
        pros::delay(100);
        rightWing.set_value(0);
    });
    //Start the slapper for match loading
    slapper.move(127);
    //delay for match loading
    pros::delay(26000);
    //stop the slapper
    slapper.move(0);
    //setup for bowl
    //move backwards and move towards close goal
    chassis.moveTo(-61, -37, 180, 800, false, false, 40, 0.5);
    //go towards the match load area and prepare to go across the ally
    chassis.moveTo(-62, -24, 180, 1000, false, false, 40, 0.2);
    //go with bot to the other side and bash alley 2 times
    chassis.moveTo(-43, -67 , 135, 800, false, true, 40, 0.4); 
    //second ram
    chassis.moveTo(-15, -67, -90, 800, false, false, 40, 0.1);
    // move towards the middle of the ally and prepare to push the triballs in 
    chassis.moveTo(34, -67, -90, 900, false, false, 40, 0.1);
    //push the triballs into the goal with the back of the bot
    chassis.moveTo(46, -67, -135, 700, false, false, 40, 0.4);
    //
    chassis.moveTo(64, -32, 180, 800, false, false, 40, 0.4);
    //go to the right side of barrier to setup for right bash
    chassis.moveTo(40, -80, -135, 700, false, true, 40, 0.4);
    chassis.moveTo(68, -28, 180, 900, false, false, 40, 0.4);
    chassis.moveTo(48, -50, -45,700, false, true, 40, 0.7);
    //bash 1
    chassis.moveTo(10, -32, -90, 1000, false, true, 40,0.2,80);
    chassis.turnTo(5,-57, 600, false, false, 70);
    leftWing.set_value(1);
    rightWing.set_value(1);
    chassis.moveTo(70, -25, -90, 1200, false, false, 40,0.7);
    //come back from bash 1
    chassis.moveTo(34, -48, -180, 1800, true, true, 40,0.3);
    chassis.waitUntilDist(10);
    leftWing.set_value(0);
    rightWing.set_value(0);
    chassis.waitUntilDist(100000);
    //go to middle
    chassis.moveTo(28, -20, 180, 1800, false, false, 40,0.6);
    chassis.turnTo(-50, 0, 500);
    //bash 2
    leftWing.set_value(1);
    rightWing.set_value(1);
    chassis.moveTo(70, -15, -90, 900, false, false, 40, 0.1);
    //come back from bash 2
    chassis.moveTo(40,-7, 90, 900, true, true, 40, 0.1,80);
    chassis.waitUntilDist(10);
    leftWing.set_value(0);
    rightWing.set_value(0);
    chassis.waitUntilDist(100000);
    //go to left side of barrier
    chassis.turnTo(0, 50, 600);
    chassis.moveTo(30, 48, -45, 800, false, true, 40, 0.2, 90);
    chassis.moveTo(80, -25, -90, 1300, true, false, 40, 0.5);
    chassis.waitUntilDist(15);
    //open the wings to push the triballs into the goal
    leftWing.set_value(1);
    rightWing.set_value(1);
    chassis.waitUntilDist(100000);
    //push triballs into the goal 
    chassis.moveTo(56, 20, 115, 700, true, true, 40, 0.3);

    chassis.waitUntilDist(10);
    //close the wings
    leftWing.set_value(0);
    rightWing.set_value(0);
    chassis.waitUntilDist(100000);
    //move towards the ends
    chassis.moveTo(85, 45, 170, 1200, false, true, 40, 0.4, 90);
    chassis.turnTo(27, 60, 700, false, false);
    
    chassis.moveTo(88, 0, 0, 1200, false, false, 40, 0.6);
    chassis.moveTo(60, 48, -45, 1200, false, true, 40, 0.2);
    chassis.moveTo(90, 0, 0, 1200, false, false, 40, 0.6);
    chassis.moveTo(85, 20, -45, 700, false, true, 40, 0.1);
    chassis.turnTo(45,58, 600, false, false);
    chassis.moveTo(30,0, 45, 1500, true, false, 40, 0.4, 127);
    chassis.waitUntilDist(20);
    leftWing.set_value(1);
    rightWing.set_value(1);
    chassis.waitUntilDist(1000000);
    chassis.moveTo(80,-35, -90, 1500, false, false, 40, 0.4, 127);
    chassis.moveTo(30,20, 45, 1500, false, true, 40, 0.4, 127);
    chassis.moveTo(30,20, 45, 1500, false, true, 40, 0.4, 127);
}
void autonomous() {
    
    tuned_skills();

    /*
    chassis.moveTo(70, 45, -90, 900, true, true, 40, 0.4);
    chassis.moveTo(80, 30, -45, 1200, false, false, 40, 0.1);
    */
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
void opcontrol() { while(true){
    
} }
