#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drive motors
pros::Motor lF(-17, pros::E_MOTOR_GEARSET_06); // left front motor. port 12, reversed
pros::Motor lM(-16, pros::E_MOTOR_GEARSET_06); // left middle motor. port 11, reversed
pros::Motor lB(-21, pros::E_MOTOR_GEARSET_06); // left back motor. port 1, reversed
pros::Motor rF(15, pros::E_MOTOR_GEARSET_06); // right front motor. port 2
pros::Motor rM(12, pros::E_MOTOR_GEARSET_06); // right middle motor. port 11
pros::Motor rB(1, pros::E_MOTOR_GEARSET_06); // right back motor. port 13

pros::Motor intake(8, pros::E_MOTOR_GEARSET_06); // right back motor. port 13
pros::Motor slapper(-5, pros::E_MOTOR_GEARSET_06); // right back motor. port 13

// motor groups
pros::MotorGroup leftMotors({lF, lM, lB}); // left motor group
pros::MotorGroup rightMotors({rF, rM, rB}); // right motor group

// pistons
pros::ADIDigitalOut wings('H');
pros::ADIDigitalOut rightWing('A');
pros::ADIDigitalOut leftWing('D');
pros::ADIDigitalOut backWings('F');
pros::ADIDigitalOut hang('B');

// Inertial Sensor on port 6
pros::Imu imu(7);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 15, reversed (negative signs don't work due to a pros bug)
pros::Rotation horizontalEnc(15, true);
// horizontal tracking wheel. 2.75" diameter, 3.7" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -3.7);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 360
                              2 // chase power is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(9, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            6.5, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1.95, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10.9, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
// note that in this example we use internal motor encoders (IMEs), so we don't pass vertical tracking wheels
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    chassis.setPose(0,0,0);

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

void moveDrive(int ms) {
    lB.move_velocity(6000);
    lM.move_velocity(6000);
    lF.move_velocity(6000);
    rB.move_velocity(6000);
    rM.move_velocity(6000);
    rF.move_velocity(6000);
    pros::delay(ms);
     lB.move_velocity(0);
    lM.move_velocity(0);
    lF.move_velocity(0);
    rB.move_velocity(0);
    rM.move_velocity(0);
    rF.move_velocity(0);
  
}

void moveDriveBackward(int ms) {
    lB.move_velocity(-6000);
    lM.move_velocity(-6000);
    lF.move_velocity(-6000);
    rB.move_velocity(-6000);
    rM.move_velocity(-6000);
    rF.move_velocity(-6000);
    pros::delay(ms);
     lB.move_velocity(0);
    lM.move_velocity(0);
    lF.move_velocity(0);
    rB.move_velocity(0);
    rM.move_velocity(0);
    rF.move_velocity(0);
  
}


void winpointauton() {
chassis.setPose(0, 0, 0);
chassis.moveToPoint(0,-6,1000);
chassis.waitUntilDone();
hang.set_value(1);
pros::delay(500);
chassis.moveToPoint(0, 0, 1000);
chassis.waitUntilDone();
chassis.turnTo(-55, 0, 1000);
chassis.waitUntilDone();
hang.set_value(0);
chassis.setPose(0,0,0);
chassis.moveToPoint(-20, 34, 1000);
chassis.waitUntilDone();
chassis.setPose(0,0,0);
chassis.waitUntilDone();
chassis.turnTo(-72, 0, 1000);
chassis.waitUntilDone();
chassis.setPose(0,0,0);
intake.move_velocity(6000);
pros::delay(400);
moveDrive(850);
chassis.moveToPoint(0,-5,1000,false);
chassis.waitUntilDone();
chassis.moveToPoint(-50,4,1500, true, 100);
chassis.waitUntilDone();
// chassis.setPose(0,0,0);
// chassis.waitUntilDone();
chassis.turnTo(-50, -95,1800);
chassis.waitUntilDone();
chassis.setPose(0,0,0);
chassis.waitUntilDone();
chassis.moveToPoint(0, 29.5, 5000);
}

void poop() {
chassis.setPose(0,0,0);
    // moveDriveBackward(700);
    chassis.moveToPoint(0,-32,1300,false, 110);
    // chassis.waitUntilDone();
    chassis.moveToPoint(1,-22.4,1000,true);
    chassis.waitUntilDone();
    chassis.turnTo(175,-59,1000,false);
    chassis.waitUntilDone();
   
     backWings.set_value(1);
    chassis.waitUntilDone();
            slapper.move_voltage(-999999999);
            pros::delay(21000);
            slapper.move_voltage(0);
    // chassis.waitUntilDone();
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    backWings.set_value(0);
    chassis.waitUntilDone();
    
    chassis.turnTo(28,69,1000);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    // run cata
    // intake.move_velocity(6000);
    chassis.moveToPoint(0, 37.5, 1050);
    chassis.waitUntilDone();

    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.turnTo(-175, 30, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    rightWing.set_value(1);
    chassis.moveToPoint(0,72, 1750,true,109);
    chassis.waitUntilDone();
    rightWing.set_value(0);
    chassis.moveToPoint(0, 68, 700,false);
    chassis.waitUntilDone();
    chassis.turnTo(0, 80, 500);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.turnTo(-90, -2, 800);
    intake.move_velocity(-6000);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,21, 800);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.turnTo(90,0,700,true);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 23.5, 1000,true);
    chassis.waitUntilDone();
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.waitUntilDone();
    chassis.turnTo(79, 3.5, 700, false, 97);
    chassis.waitUntilDone();
    chassis.moveToPoint(72, 2.5, 3500, false, 97);
    chassis.waitUntilDone();
    chassis.moveToPoint(148, -65, 1000,false);
    chassis.waitUntilDone();
    chassis.turnTo(108, -65, 1000,false);
    chassis.waitUntilDone();
    moveDriveBackward(400);
    leftWing.set_value(0);
    chassis.waitUntilDone();
    chassis.moveToPoint(112, -3.5, 1000,true);
    chassis.waitUntilDone();
    chassis.turnTo(112, -55, 1000,true);
    chassis.waitUntilDone();
    chassis.setPose(0, -5, 0);
    chassis.waitUntilDone();
}

void skills() {
  
    poop();
    chassis.waitUntilDone();
    chassis.turnTo(40,7,1000,true);
    chassis.waitUntilDone();
    // chassis.setPose(0,0,0);
    // chassis.waitUntilDone();
    chassis.moveToPoint(35, 7, 1000, true, 80);
    chassis.waitUntilDone();
    chassis.moveToPoint(35, 20, 800, true);
    chassis.waitUntilDone();
    chassis.turnTo(8, 18, 1000);
    chassis.waitUntilDone();
rightWing.set_value(1);
// leftWing.set_value(1);
    
    chassis.moveToPoint(8, 24, 1000);
    chassis.waitUntilDone();
    chassis.turnTo(0, 17, 700,true);
    // chassis.moveToPoint(-5, 18, 900);
    moveDrive(700);
    rightWing.set_value(0);
    pros::delay(100);
    // leftWing.set_value(0);
    chassis.waitUntilDone();
    chassis.moveToPoint(32, 16, 800, false, 80);
    chassis.waitUntilDone();
    chassis.turnTo(30, 60, 700,true);
    chassis.waitUntilDone();
    chassis.moveToPoint(30, 62, 1000,true);
    chassis.waitUntilDone();
    chassis.turnTo(0, 38, 700,true);
    chassis.waitUntilDone();
    leftWing.set_value(1);
    rightWing.set_value(1);
    chassis.waitUntilDone();

    moveDrive(1000);
    chassis.waitUntilDone();
    leftWing.set_value(0);
    chassis.moveToPoint(36, 56, 1000,false);
    chassis.waitUntilDone();
    rightWing.set_value(1);
    pros::delay(300);
    backWings.set_value(0);
    chassis.waitUntilDone();
    chassis.turnTo(-3, 91, 700);
    chassis.waitUntilDone();
    chassis.moveToPoint(-3, 85, 1200, true, 90);
    chassis.waitUntilDone();
    chassis.moveToPoint(-37, 61, 1000, true);
    chassis.waitUntilDone();
    chassis.turnTo(-24, 61, 700, true);
    chassis.waitUntilDone();
    rightWing.set_value(0);
    moveDrive(500);
    chassis.waitUntilDone();
    chassis.moveToPoint(-37, 68, 1000, false);
    chassis.waitUntilDone();
    chassis.turnTo(-24, 61, 1000, false);
    chassis.waitUntilDone();
    moveDriveBackward(800);
    chassis.waitUntilDone();
    chassis.turnTo(12, 105, 700, true);
    chassis.moveToPoint(12, 105, 1000, true);
    chassis.waitUntilDone();
    chassis.turnTo(54, 96, 1000);
    hang.set_value(true);
    chassis.waitUntilDone();
    chassis.moveToPoint(60, 99, 1000, true, 100);
    chassis.waitUntilDone();
    hang.set_value(false);












    // chassis.setPose(0, 0, 0);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(-24, 24, 1000);
    // chassis.waitUntilDone();
    // chassis.turnTo(0, 0, 1000, false);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(-5, -12, 1000, false, 100);
    // chassis.waitUntilDone();
    // chassis.setPose(0,0,0);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(0, 24, 1000);
    // chassis.waitUntilDone();
    // rightWing.set_value(1);
    // chassis.moveToPoint(-40, -16, 1000, true, 100);
    // chassis.waitUntilDone();
    // chassis.turnTo(-50, 0, 1000);



    // chassis.moveToPoint(0, 10, 1000);
    // chassis.waitUntilDone();
    // rightWing.set_value(1);
    // chassis.moveToPoint(-24, -10, 1000);
    // chassis.waitUntilDone();
    // chassis.turnTo(40, -10, 1000);
    

    // chassis.waitUntilDone();
    // chassis.turnTo(70, -20, 1000,false);
    
    
    // chassis.turnTo(37,25,1000,true);
    // chassis.waitUntilDone();
    // chassis.waitUntilDone();
    // chassis.turnTo(37,25,1000,true);

//     chassis.turnTo(-65,0,1000,false);
//     chassis.waitUntilDone();
//     moveDriveBackward(700);
//     chassis.waitUntilDone();
//     chassis.setPose(0,0,0);
//     chassis.waitUntilDone();
// chassis.moveToPoint(-8, 12, 2000,true);
// chassis.waitUntilDone();
//     chassis.turnTo(-28, -6, 1000);
//     chassis.waitUntilDone();
//     chassis.moveToPoint(-28,-28,1000);
//     chassis.waitUntilDone();
//     chassis.turnTo(-12, 34, 1000);
// chassis.waitUntilDone();
    // moveDriveBackward(700);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(80, 3, 2000,true);
    // chassis.waitUntilDone();
    // chassis.turnTo(142,-8,1000,false);

    // moveDriveBackward(800);
    // chassis.waitUntilDone();
    // chassis.waitUntilDone();
    // chassis.moveToPoint(105, -12, 2000,true);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(145, -24, 2000,false);
    //     moveDriveBackward(1000);
    // chassis.turnTo(20, 0, 1000, true);
    // chassis.waitUntilDone();
    // chassis.setPose(0, 0, 0);
    // chassis.waitUntilDone();
    // leftWing.set_value(1);
    // chassis.moveToPoint(-5, 105, 1500,true);
    // // chassis.turnTo(35,-18,1000,true);
    // chassis.waitUntilDone();
    // intake.move_velocity(6000);
    // chassis.moveToPoint(30, 110, 1500,true, 100);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(10, 110, 1500,true, 100);

    // chassis.waitUntilDone();
    // chassis.moveToPoint(12, 110, 1000,false);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(30, 110, 1500,true);
    // moveDrive(1000);

    

}







void realsix() {
    chassis.setPose(0,0,0);
    intake.move_velocity(-6000);
    pros::delay(20);
    chassis.moveToPoint(0,4,1000,true);
    chassis.waitUntilDone();
    intake.move_velocity(0);
    chassis.moveToPoint(4, -35, 900,false,110);
    chassis.waitUntilDone();
    backWings.set_value(1);
    chassis.moveToPoint(10, -53, 1100,false,85);
    chassis.waitUntilDone();
    chassis.turnTo(100, -53, 700, false);
backWings.set_value(0);
    chassis.waitUntilDone();
    moveDriveBackward(650);
    chassis.waitUntilDone();
chassis.moveToPoint(16, -59, 800,true);
    chassis.waitUntilDone();
    chassis.turnTo(100, -59, 700, true);
    chassis.waitUntilDone();
    rightWing.set_value(1);
    
    intake.move_velocity(6000);
    pros::delay(400);
    moveDrive(400);
    chassis.waitUntilDone();
    rightWing.set_value(0);
    intake.move_velocity(0);
chassis.moveToPoint(16, -59, 800,false);
    chassis.waitUntilDone();
     chassis.turnTo(40,  -4, 500, true, 90);
     intake.move_velocity(-6000);
     chassis.moveToPoint(40,  -4, 1000, true, 90);
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(80,  -29, 1000, true, 70);
    chassis.waitUntilDone();
    chassis.turnTo(40, -60, 500);
    chassis.waitUntilDone();
    intake.move_velocity(6000);
    pros::delay(200);
    leftWing.set_value(1);
    moveDrive(700);
    rightWing.set_value(0);
    leftWing.set_value(0);
    pros::delay(300);
    chassis.setPose(0,0,0);
    intake.move_velocity(-6000);
    chassis.moveToPoint(-8, -47, 1800);
    pros::delay(500);
    chassis.waitUntilDone();
    chassis.turnTo(0,0,1000);
    chassis.waitUntilDone();
    rightWing.set_value(1);
    leftWing.set_value(1);
    intake.move_velocity(6000);
    pros::delay(200);
    moveDrive(1000);
}

void newwinpoint() {
    chassis.setPose(0,0,0);
    chassis.turnTo(-15,6,1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-15, 6, 1700,true,70);
    chassis.waitUntilDone();
    backWings.set_value(1);
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.turnTo(90, 40, 800,false,150);
    chassis.waitUntilDone();
    backWings.set_value(0);
    pros::delay(500);
    chassis.turnTo(0, -12, 800,true,150);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, -12, 800,true,120);
    chassis.waitUntilDone();
    chassis.turnTo(35.5, -12, 800,true,120);
    chassis.waitUntilDone();
    chassis.moveToPoint(33.5, -12, 2000,true,60);
    intake.move_velocity(6000);
    chassis.waitUntilDone();
}


void sixball() {
    chassis.setPose(0,0,0);
    intake.move_velocity(-600);
    pros::delay(20);
    chassis.moveToPoint(0,4,1000,true);
    chassis.waitUntilDone();
    intake.move_velocity(0);
    chassis.moveToPoint(16, -43, 3000,false,110);
    chassis.waitUntilDone();
    chassis.moveToPoint(35, -52, 1200,false,110);
    chassis.waitUntilDone();
    chassis.moveToPoint(27, -55, 1000,false,110);
    chassis.waitUntilDone();
    chassis.turnTo(42,-50,700);
    chassis.waitUntilDone();
    intake.move_velocity(6000);
    pros::delay(700);
    moveDrive(700);
    intake.move_velocity(0);
    chassis.moveToPoint(28, -48, 1200,false,110);
    intake.move_velocity(-6000);
    chassis.moveToPoint(40,  -4, 1000, true, 90);
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(85,  -29, 1000, true, 70);
    chassis.waitUntilDone();
    wings.set_value(1);
    chassis.turnTo(60, -60, 500);
    chassis.waitUntilDone();
    intake.move_velocity(6000);
    pros::delay(200);
    moveDrive(540);
    wings.set_value(0);
    pros::delay(500);
    // chassis.setPose(0,0,0);
    // intake.move_velocity(-6000);
    // chassis.moveToPoint(-6, -44, 1000);
    // pros::delay(1200);
    // chassis.waitUntilDone();
    // chassis.turnTo(-10,0,1000);
    // chassis.waitUntilDone();
    // wings.set_value(1);
    // intake.move_velocity(6000);
    // pros::delay(200);
    // moveDrive(1000);



    // chassis.moveToPoint(60,-1,2000, 75);
    // chassis.waitUntilDone();
    // pros::delay(200);
    // chassis.waitUntilDone();
    // chassis.turnTo(80, -60, 500);
    // intake.move_velocity(0);
    // chassis.waitUntilDone();
    // wings.set_value(1);
    // intake.move_velocity(6000);
    // moveDrive(1000);
    // wings.set_value(1);
    

    // wings.set_value(1);
    // chassis.moveToPoint(68, -40, 1000);
    // chassis.moveToPoint()
    // chassis.waitUntilDone();
    // intake.move_velocity(6000);
    // pros::delay(500);
    // chassis.turnTo(34, -48, 1000, false);
    // chassis.waitUntilDone();
    // moveDriveBackward(1000);
    // moveDrive(1000);
    // chassis.moveToPoint(28, -48, 1000);
}



void fourball() {
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 38.5, 1000);
    chassis.waitUntilDone();
    chassis.waitUntilDone();
    chassis.turnTo(90, 38.5, 1000);
    chassis.waitUntilDone();
    intake.move_velocity(6000);
    pros::delay(500);
    moveDrive(700);
    intake.move_velocity(0);
    pros::delay(50);
    intake.move_velocity(-6000);
    chassis.moveToPoint(-39,10,1000);
    chassis.waitUntilDone();
    chassis.turnTo(-32,10,1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(3, 42, 1000, true, 100);
    chassis.waitUntilDone();
    chassis.turnTo(90,42,1000);
    // chassis.waitUntilDone();
    wings.set_value(1);
    // chassis.waitUntilDone();


}



void disruptor() {
    chassis.setPose(0, 0, 0);
    // intake.move_velocity(-600);
    chassis.moveToPoint(7, 40, 1000);
    chassis.turnTo(33,44,1000);
    wings.set_value(1);
    chassis.waitUntilDone();
    // moveDrive(300);
    chassis.moveToPoint(30,40,1000);

    pros::delay(1000);
    wings.set_value(0);
    chassis.moveToPoint(20,33,1000, true);
    chassis.turnTo(0,37,1000, true);
    chassis.waitUntilDone();
    intake.move_velocity(-600);
    pros::delay(500);
    moveDrive(600);
    pros::delay(500);
    chassis.moveToPoint(18,33,1000, true);
    chassis.waitUntilDone();
    // chassis.moveToPoint(20,30,1000, true);
    chassis.moveToPoint(0,-17,1000, true);
    // chassis.waitUntilDone();
    chassis.turnTo(40,-4,1000,true);
    chassis.waitUntilDone();
    intake.move_velocity(-600);
    pros::delay(10);
    chassis.moveToPoint(40,0,1000,true);
    chassis.moveToPoint(-24,0,1000,false);

    // chassis.moveToPoint(12, 28, 1000);
    // chassis.moveToPoint(0,-12,1000);
    // chassis.turnTo(10, 0, 1000);
    // intake.move_velocity(-600);
    // chassis.moveToPoint(24,-12,1000);
    // chassis.moveToPoint(-24,0,1000);
    // chassis.moveToPoint(10,0,1000);
    // chassis.moveToPoint(0,-5,1000);
}



/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
// winpointauton();
// disruptor();
// sixball();
// skills();
// realsix();
// newwinpoint();


  



//   chassis.setPose(0,0,0);
//     // moveDriveBackward(700);
//     chassis.moveToPoint(0,-32,1400,false, 80);
//     // chassis.waitUntilDone();
//     chassis.moveToPoint(0,-24,1000,true);
//     chassis.waitUntilDone();
//     chassis.turnTo(175,-19,2500,false);
//     backWings.set_value(1);
//     chassis.waitUntilDone();
//             slapper.move_voltage(-95000000);
//     chassis.setPose(0,0,0);
    // // chassiAs.waitUntilDone();
    // chassis.waitUntilDone();
    // chassis.setPose(0,0,0);
    // chassis.waitUntilDone();
    
    // chassis.turnTo(57,69,1000);
    // chassis.waitUntilDone();
    // chassis.setPose(0,0,0);
    // chassis.waitUntilDone();


// fourball();
realsix();
}

/**
 * Runs in driver control
 */
void opcontrol() {
// chassis.setPose(0,0,0);
//     chassis.moveToPoint(0,-32,1300,false, 80);
//     chassis.moveToPoint(1,-22.5,1000,true);
//     chassis.waitUntilDone();
//     chassis.turnTo(175,-59,1000,false);
//     chassis.waitUntilDone();
//     backWings.set_value(1);
//             slapper.move_voltage(-999999999);
//             pros::delay(21000);
static bool leftwingtoggle { false };
static bool rightwingtoggle { false };
static bool toggle { false };
static bool backwingtoggle { false };
static bool hangToggle { false };
      // controller
    // loop to continuously update motors
    while (true) {
        bool buttonPressed = false;
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    // if (controller.get_analog(pros::E_CONTROLLER_DIGITAL_R1))

        // move the chassis with curvature drive
        chassis.tank(leftY, rightX);



if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
    if (!leftwingtoggle) {
        leftWing.set_value(1);
        leftwingtoggle = !leftwingtoggle;
    }
    else {
        leftWing.set_value(0);
        leftwingtoggle = !leftwingtoggle;
    }
}


if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
    if (!backwingtoggle) {
        backWings.set_value(1);
        backwingtoggle = !backwingtoggle;
    }
    else {
        backWings.set_value(0);
        backwingtoggle = !backwingtoggle;
    }
}

if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
    if (!rightwingtoggle) {
        rightWing.set_value(1);
        rightwingtoggle = !rightwingtoggle;
    }
    else {
        rightWing.set_value(0);
        rightwingtoggle = !rightwingtoggle;
    }
}

if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
    if (!hangToggle) {
        hang.set_value(1);
        hangToggle = !hangToggle;
    }
    else {
        hang.set_value(0);
        hangToggle = !hangToggle;
    }
}

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move_voltage(12000);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move_voltage(-12000);
        } else {
            intake.move_velocity(0);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            slapper.move_voltage(-999999999);
        } else {
            slapper.move_voltage(0);
        }
        // delay to save resources
        pros::delay(10);
    }
}
