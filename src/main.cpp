#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <string>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drive motors
pros::Optical colorsort(9);
pros::Motor lF(-8, pros::E_MOTOR_GEARSET_06); // left front motor. port 12, reversed
pros::Motor lM(-3, pros::E_MOTOR_GEARSET_06); // left middle motor. port 11, reversed
pros::Motor lB(-17, pros::E_MOTOR_GEARSET_06); // left back motor. port 1, reversed
pros::Motor rF(5, pros::E_MOTOR_GEARSET_06); // right front motor. port 2
pros::Motor rM(1, pros::E_MOTOR_GEARSET_06); // right middle motor. port 11
pros::Motor rB(15, pros::E_MOTOR_GEARSET_06); // right back motor. port 13
pros::Motor leftside(7, pros::E_MOTOR_GEARSET_06); // right back motor. port 13
pros::Motor rightside(2, pros::E_MOTOR_GEARSET_06); // right back motor. port 13
pros::Rotation rotationalSensor(6);

pros::Motor intake(11, pros::E_MOTOR_GEARSET_06); // right back motor. port 13
pros::Motor intakefirst(21, pros::E_MOTOR_GEARSET_06); // right back motor. port 13
// pros::Motor slapper(-5, pros::E_MOTOR_GEARSET_06); // right back motor. port 13

// motor groups
pros::MotorGroup leftMotors({lF, lM, lB}); // left motor group
pros::MotorGroup rightMotors({rF, rM, rB}); // right motor group

// pistons
pros::ADIDigitalOut wings('C');
pros::ADIDigitalOut rightWing('Z');
pros::ADIDigitalOut clamper('D');
pros::ADIDigitalOut intakePiston('D'); // used to be C
pros::ADIDigitalOut backWings('F');
pros::ADIDigitalOut hang('B');

// Inertial Sensor on port 6
pros::Imu imu(10);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 15, reversed (negative signs don't work due to a pros bug)
pros::Rotation horizontalEnc(5, true);
// horizontal tracking wheel. 2.75" diameter, 3.7" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_325, -3.7);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 360
                              2 // chase power is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1.97, // proportional gain (kP)
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

bool auton_done = false;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    chassis.setPose(0, 0, 0);

    rightside.tare_position();
    rotationalSensor.reset_position();

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
void moveLiftToAngle(double targetAngle) {
    rightside.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    if (rotationalSensor.get_position() < targetAngle) {
        // Move up
        rightside.move_velocity(120000);
        while (rotationalSensor.get_position() < targetAngle) {
            pros::delay(1);
        }
    } else {
        // Move down
        rightside.move_velocity(-120000);
        while (rotationalSensor.get_position() > targetAngle) {
            pros::delay(1);
        }
    }
    rightside.move_velocity(0);  // Stop motor when target is reached
}

void moveLiftToPos(double targetPosition, int speed) {
    if (rotationalSensor.get_position() < targetPosition) {
        // Move up
        rightside.move_velocity(speed);
        while (rotationalSensor.get_position() < targetPosition) {
            pros::delay(10);
        }
    } else {
        // Move down
        rightside.move_velocity(-speed);
        while (rotationalSensor.get_position() > targetPosition) {
            pros::delay(10);
        }
    }
    rightside.move_velocity(0);  // Stop motor when target is reached
}

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

void moveLiftToPosition(int targetPosition, int velocity) {
    rightside.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    rightside.move_absolute(targetPosition, velocity);
    while (std::abs(rightside.get_position() - targetPosition) > 3) { pros::delay(10); }
}

void winpointauton() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, -6, 1000);
    chassis.waitUntilDone();
    hang.set_value(1);
    pros::delay(500);
    chassis.moveToPoint(0, 0, 1000);
    chassis.waitUntilDone();
    chassis.turnTo(-55, 0, 1000);
    chassis.waitUntilDone();
    hang.set_value(0);
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(-20, 34, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.waitUntilDone();
    chassis.turnTo(-72, 0, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    intake.move_velocity(6000);
    pros::delay(400);
    moveDrive(850);
    chassis.moveToPoint(0, -5, 1000, false);
    chassis.waitUntilDone();
    chassis.moveToPoint(-50, 4, 1500, true, 100);
    chassis.waitUntilDone();
    // chassis.setPose(0,0,0);
    // chassis.waitUntilDone();
    chassis.turnTo(-50, -95, 1800);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 29.5, 5000);
}

void poop() {
    chassis.setPose(0, 0, 0);
    // moveDriveBackward(700);
    chassis.moveToPoint(0, -32, 1300, false, 110);
    // chassis.waitUntilDone();
    chassis.moveToPoint(1, -22.4, 1000, true);
    chassis.waitUntilDone();
    chassis.turnTo(175, -59, 1000, false);
    chassis.waitUntilDone();

    backWings.set_value(1);
    chassis.waitUntilDone();
    // slapper.move_voltage(-999999999);
    // pros::delay(21000);
    // slapper.move_voltage(0);
    // chassis.waitUntilDone();
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    backWings.set_value(0);
    chassis.waitUntilDone();

    chassis.turnTo(28, 69, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.waitUntilDone();
    // run cata
    // intake.move_velocity(6000);
    chassis.moveToPoint(0, 37.5, 1050);
    chassis.waitUntilDone();

    chassis.setPose(0, 0, 0);
    chassis.waitUntilDone();
    chassis.turnTo(-175, 30, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.waitUntilDone();
    rightWing.set_value(1);
    chassis.moveToPoint(0, 72, 1750, true, 109);
    chassis.waitUntilDone();
    rightWing.set_value(0);
    chassis.moveToPoint(0, 68, 700, false);
    chassis.waitUntilDone();
    chassis.turnTo(0, 80, 500);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.waitUntilDone();
    chassis.turnTo(-90, -2, 800);
    intake.move_velocity(-6000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 21, 800);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.waitUntilDone();
    chassis.turnTo(90, 0, 700, true);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 23.5, 1000, true);
    chassis.waitUntilDone();
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.waitUntilDone();
    chassis.turnTo(79, 3.5, 700, false, 97);
    chassis.waitUntilDone();
    chassis.moveToPoint(72, 2.5, 3500, false, 97);
    chassis.waitUntilDone();
    chassis.moveToPoint(148, -65, 1000, false);
    chassis.waitUntilDone();
    chassis.turnTo(108, -65, 1000, false);
    chassis.waitUntilDone();
    moveDriveBackward(400);
    chassis.waitUntilDone();
    chassis.moveToPoint(112, -3.5, 1000, true);
    chassis.waitUntilDone();
    chassis.turnTo(112, -55, 1000, true);
    chassis.waitUntilDone();
    chassis.setPose(0, -5, 0);
    chassis.waitUntilDone();
}

void skills() {
    poop();
    chassis.waitUntilDone();
    chassis.turnTo(40, 7, 1000, true);
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
    chassis.turnTo(0, 17, 700, true);
    // chassis.moveToPoint(-5, 18, 900);
    moveDrive(700);
    rightWing.set_value(0);
    pros::delay(100);
    // leftWing.set_value(0);
    chassis.waitUntilDone();
    chassis.moveToPoint(32, 16, 800, false, 80);
    chassis.waitUntilDone();
    chassis.turnTo(30, 60, 700, true);
    chassis.waitUntilDone();
    chassis.moveToPoint(30, 62, 1000, true);
    chassis.waitUntilDone();
    chassis.turnTo(0, 38, 700, true);
    chassis.waitUntilDone();
    rightWing.set_value(1);
    chassis.waitUntilDone();

    moveDrive(1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(36, 56, 1000, false);
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
    chassis.setPose(0, 0, 0);
    intake.move_velocity(-6000);
    pros::delay(20);
    chassis.moveToPoint(0, 4, 1000, true);
    chassis.waitUntilDone();
    intake.move_velocity(0);
    chassis.moveToPoint(4, -35, 900, false, 110);
    chassis.waitUntilDone();
    backWings.set_value(1);
    chassis.moveToPoint(10, -53, 1100, false, 85);
    chassis.waitUntilDone();
    chassis.turnTo(100, -53, 700, false);
    backWings.set_value(0);
    chassis.waitUntilDone();
    moveDriveBackward(650);
    chassis.waitUntilDone();
    chassis.moveToPoint(16, -59, 800, true);
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
    chassis.moveToPoint(16, -59, 800, false);
    chassis.waitUntilDone();
    chassis.turnTo(40, -4, 500, true, 90);
    intake.move_velocity(-6000);
    chassis.moveToPoint(40, -4, 1000, true, 90);
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(80, -29, 1000, true, 70);
    chassis.waitUntilDone();
    chassis.turnTo(40, -60, 500);
    chassis.waitUntilDone();
    intake.move_velocity(6000);
    pros::delay(200);
    moveDrive(700);
    rightWing.set_value(0);
    pros::delay(300);
    chassis.setPose(0, 0, 0);
    intake.move_velocity(-6000);
    chassis.moveToPoint(-8, -47, 1800);
    pros::delay(500);
    chassis.waitUntilDone();
    chassis.turnTo(0, 0, 1000);
    chassis.waitUntilDone();
    rightWing.set_value(1);
    intake.move_velocity(6000);
    pros::delay(200);
    moveDrive(1000);
}

void newwinpoint() {
    chassis.setPose(0, 0, 0);
    chassis.turnTo(-15, 6, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-15, 6, 1700, true, 70);
    chassis.waitUntilDone();
    backWings.set_value(1);
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.turnTo(90, 40, 800, false, 150);
    chassis.waitUntilDone();
    backWings.set_value(0);
    pros::delay(500);
    chassis.turnTo(0, -12, 800, true, 150);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, -12, 800, true, 120);
    chassis.waitUntilDone();
    chassis.turnTo(35.5, -12, 800, true, 120);
    chassis.waitUntilDone();
    chassis.moveToPoint(33.5, -12, 2000, true, 60);
    intake.move_velocity(6000);
    chassis.waitUntilDone();
}

void sixball() {
    chassis.setPose(0, 0, 0);
    intake.move_velocity(-600);
    pros::delay(20);
    chassis.moveToPoint(0, 4, 1000, true);
    chassis.waitUntilDone();
    intake.move_velocity(0);
    chassis.moveToPoint(16, -43, 3000, false, 110);
    chassis.waitUntilDone();
    chassis.moveToPoint(35, -52, 1200, false, 110);
    chassis.waitUntilDone();
    chassis.moveToPoint(27, -55, 1000, false, 110);
    chassis.waitUntilDone();
    chassis.turnTo(42, -50, 700);
    chassis.waitUntilDone();
    intake.move_velocity(6000);
    pros::delay(700);
    moveDrive(700);
    intake.move_velocity(0);
    chassis.moveToPoint(28, -48, 1200, false, 110);
    intake.move_velocity(-6000);
    chassis.moveToPoint(40, -4, 1000, true, 90);
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(85, -29, 1000, true, 70);
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
    chassis.setPose(0, 0, 0);
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
    chassis.moveToPoint(-39, 10, 1000);
    chassis.waitUntilDone();
    chassis.turnTo(-32, 10, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(3, 42, 1000, true, 100);
    chassis.waitUntilDone();
    chassis.turnTo(90, 42, 1000);
    // chassis.waitUntilDone();
    wings.set_value(1);
    // chassis.waitUntilDone();
}

void disruptor() {
    chassis.setPose(0, 0, 0);
    // intake.move_velocity(-600);
    chassis.moveToPoint(7, 40, 1000);
    chassis.turnTo(33, 44, 1000);
    wings.set_value(1);
    chassis.waitUntilDone();
    // moveDrive(300);
    chassis.moveToPoint(30, 40, 1000);

    pros::delay(1000);
    wings.set_value(0);
    chassis.moveToPoint(20, 33, 1000, true);
    chassis.turnTo(0, 37, 1000, true);
    chassis.waitUntilDone();
    intake.move_velocity(-600);
    pros::delay(500);
    moveDrive(600);
    pros::delay(500);
    chassis.moveToPoint(18, 33, 1000, true);
    chassis.waitUntilDone();
    // chassis.moveToPoint(20,30,1000, true);
    chassis.moveToPoint(0, -17, 1000, true);
    // chassis.waitUntilDone();
    chassis.turnTo(40, -4, 1000, true);
    chassis.waitUntilDone();
    intake.move_velocity(-600);
    pros::delay(10);
    chassis.moveToPoint(40, 0, 1000, true);
    chassis.moveToPoint(-24, 0, 1000, false);

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


void ladyBrownUp(int ms) {
    rightside.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    intakefirst.move_velocity(500);
    pros::delay(80);
    intakefirst.move_velocity(0);
    rightside.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    rightside.move_voltage(-12000);
    pros::delay(ms);
    rightside.move_voltage(0);   
}


void redallianceonestake() {
    auton_done = true;
    chassis.setPose(0, 0, 0);
    rightside.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveToPoint(0, 3.25, 1000);
    chassis.waitUntilDone();
    rightside.move_voltage(-12000);
    pros::delay(750);
    rightside.move_voltage(0);
    pros::delay(100);
    chassis.moveToPoint(1, -3, 1000, false);
    chassis.turnTo(6, 1, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.waitUntilDone();
    chassis.moveToPoint(2.8, -22.2, 4200, false, 60);
    chassis.waitUntilDone();
    clamper.set_value(1);
    pros::delay(250);
    intake.move_voltage(-12000);
    intakefirst.move_voltage(-12000);
    chassis.turnTo(24, -46, 1000, true, 100);
    rightside.move_velocity(12000);
    pros::delay(1000);
    rightside.move_velocity(0);
    chassis.moveToPoint(11.2, -32.9, 1000, true);
    rightside.tare_position();
    chassis.waitUntilDone();
    pros::delay(650);
    chassis.turnTo(20, -32.25, 800);
    chassis.waitUntilDone();
    chassis.moveToPoint(17.4, -32.25, 1000);
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(1, -21, 1000, false);
    chassis.turnTo(24, -21, 1000);
    chassis.moveToPoint(7.7, -21, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, -16, 2000, false, 95);
    chassis.waitUntilDone();
    chassis.moveToPoint(-21, -13, 2000, true, 95);
    chassis.waitUntilDone();
}








void redrush() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(2, 19, 2000);
    chassis.waitUntilDone();
    hang.set_value(1);
    pros::delay(250);
    chassis.moveToPoint(2, 12, 2000);
    chassis.waitUntilDone();
    hang.set_value(0);
    pros::delay(250);
    chassis.turnTo(-6, 9.5, 1000, false);
    chassis.waitUntilDone();
    chassis.moveToPoint(-9, 9.5, 1000, false);
    chassis.waitUntilDone();
    clamper.set_value(1);
    pros::delay(250);
    chassis.waitUntilDone();
    intake.move_voltage(-12000);
    intakefirst.move_voltage(-12000);
    chassis.moveToPoint(12, 12, 1000);
    chassis.waitUntilDone();
    chassis.turnTo(12, 0, 1000, false);
    chassis.waitUntilDone();
    clamper.set_value(0);
    chassis.moveToPoint(-3, 10, 1000, true);
    chassis.waitUntilDone();
    chassis.turnTo(3, 12, 1000, false);
    chassis.waitUntilDone();
}

void redallianceonestakeelims() {
    // trigger to stop intake
    bool runIntake = true;
    // initial hue
    double hue = -1;
    while (true) {
        // constantly check color value
        hue = colorsort.get_hue();

        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, 10.4, 1000);
        chassis.waitUntilDone();
        chassis.moveToPoint(0, 2.6, 1000, false);
        chassis.waitUntilDone();
        chassis.turnTo(93, -2, 1000, false);
        chassis.waitUntilDone();
        chassis.setPose(0, 0, 0);
        chassis.waitUntilDone();
        chassis.moveToPoint(0, -3.9, 1000, false);

        // if color detected
        if (!runIntake) {
            intake.move_velocity(0);
        } else if (hue >= 180 && hue <= 240) {
            // wait for some time to release ring
            runIntake = false;
            intake.move_voltage(-12000);
            pros::delay(280);
            intake.move_velocity(0);
            pros::delay(400);
            runIntake = true;
        } else {
            intake.move_voltage(-12000);
        }
        pros::delay(500);
        chassis.moveToPoint(0, 2, 1000, false);

        chassis.waitUntilDone();
        chassis.turnTo(-17, 18, 1000, false);
        chassis.waitUntilDone();
        chassis.moveToPoint(-17, 18.5, 1700, false, 70);

        chassis.waitUntilDone();
        clamper.set_value(1);

        pros::delay(250);
        chassis.moveToPoint(-17, 24, 1000, false);
        chassis.waitUntilDone();

        chassis.turnTo(-29, 24, 1000, true);

        chassis.moveToPoint(-38, 24, 1000, 107);
        pros::delay(200);
        chassis.moveToPoint(-32, 24, 1000);

        chassis.turnTo(-32, 36, 1000);

        // chassis.turnTo(-40, 33, 1000, 50);
        chassis.waitUntilDone();
        chassis.moveToPoint(-32, 35, 1000);
        pros::delay(1600);
        chassis.moveToPoint(-32, 18, 1000, false);
        chassis.moveToPoint(50, -14, 10000, false, 110);
    }
}

void redalliancetwostake() {
    chassis.setPose(0, 0, 0);
    rightside.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveToPoint(0, 2.95, 1000);
    chassis.waitUntilDone();
    rightside.move_voltage(-12000);
    pros::delay(750);
    rightside.move_voltage(0);
    pros::delay(100);
    chassis.moveToPoint(0, -3, 1000, false);
    chassis.turnTo(-6, 0, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(-5, -20, 2400, false, 50);
    chassis.waitUntilDone();
    clamper.set_value(1);
    pros::delay(250);
    intake.move_velocity(-12000);
    intakefirst.move_voltage(-12000);

    chassis.turnTo(-48, -20, 1000, true, 80);
    chassis.waitUntilDone();
    rightside.move_velocity(120000);
    pros::delay(300);
    rightside.move_velocity(0);
    chassis.moveToPoint(-20.7, -24, 1000, true, 90);
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.moveToPoint(-16, -12, 1000, false);
    chassis.waitUntilDone();
    clamper.set_value(0);
    pros::delay(200);
    chassis.moveToPoint(-18, -12, 1000, true);
    chassis.waitUntilDone();
    chassis.turnTo(-23.8, -36, 1000, false);
    chassis.waitUntilDone();
    chassis.moveToPoint(-23.8, -23.8, 3000, false, 60);
}



void redalliancetwostakestop() {
    chassis.setPose(0, 0, 0);
    rightside.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pros::delay(4000);
    chassis.moveToPoint(0, 4, 1000);
    chassis.waitUntilDone();
    rightside.move_voltage(-12000);
    pros::delay(750);
    rightside.move_voltage(0);
    pros::delay(100);
    chassis.moveToPoint(0, -3, 1000, false);
    chassis.turnTo(-6, 0, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(-6, -20, 2400, false, 50);
    chassis.waitUntilDone();
    clamper.set_value(1);
    pros::delay(250);
    intake.move_velocity(-12000);
    intakefirst.move_voltage(-12000);
    chassis.turnTo(-48, -20, 1000, true, 80);
    chassis.waitUntilDone();
    rightside.move_velocity(120000);
    pros::delay(300);
    rightside.move_velocity(0);
    chassis.moveToPoint(-21, -24, 1000, true, 90);
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.turnTo(-21, 0, 1000, true);
    chassis.waitUntilDone();
    pros::delay(900);
    chassis.moveToPoint(-21, -8, 2000, true, 90);
            intake.move_velocity(0);
    intakefirst.move_voltage(0);
    	chassis.waitUntilDone();
    hang.set_value(1);
    chassis.waitUntilDone();
    chassis.turnTo(-23,-36,1000);
    chassis.waitUntilDone();
    pros::delay(400);
    hang.set_value(0);
    clamper.set_value(0);
    // chassis.moveToPoint(-18, -12, 1000, true);
    chassis.turnTo(-13.8, -36, 1000, false);
    chassis.waitUntilDone();
    chassis.moveToPoint(-13.8, -23.8, 3000, false, 60);
}

void blueallianceonestake() {
    chassis.setPose(0, 0, 0);
    rightside.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveToPoint(0, 3.07, 1000);
    chassis.waitUntilDone();
    rightside.move_voltage(-12000);
    pros::delay(750);
    rightside.move_voltage(0);
    pros::delay(100);
    chassis.moveToPoint(0, -3, 1000, false);
    chassis.turnTo(-6, 0.5, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.waitUntilDone();
    chassis.moveToPoint(-6.4, -21, 2800, false, 60);
    chassis.waitUntilDone();
    clamper.set_value(1);
    pros::delay(150);
    chassis.turnTo(-27, -46, 1000, true);
    chassis.waitUntilDone();
    rightside.move_velocity(12000);
    pros::delay(1100);
    rightside.move_velocity(0);
    intake.move_voltage(-12000);
    intakefirst.move_voltage(-12000);
    chassis.moveToPoint(-20.1, -35, 1000, true);
    chassis.waitUntilDone();
    pros::delay(750);
    chassis.turnTo(-30, -34.3, 800);
    chassis.moveToPoint(-26.9, -34.3, 1000);
    pros::delay(900);
    chassis.waitUntilDone();
    chassis.moveToPoint(-5, -22.5, 1000, false);
    chassis.turnTo(-24, -24, 1000);
    chassis.moveToPoint(-12, -22.5, 1000);
    chassis.waitUntilDone();
    pros::delay(200);
    chassis.moveToPoint(0, -18, 1000, false, 95);
    chassis.waitUntilDone();
    chassis.moveToPoint(22, -25, 1000, true, 95);
    chassis.waitUntilDone();
}

void bluealliancetwostake() {
    chassis.setPose(0, 0, 0);
    rightside.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveToPoint(0, 3.3, 1000);
    chassis.waitUntilDone();
    rightside.move_voltage(-12000);
    pros::delay(750);
    rightside.move_voltage(0);
    pros::delay(100);
    chassis.moveToPoint(0, -3, 1000, false);
    chassis.turnTo(6, 0, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(3.5, -21.5, 2700, false, 47);
    chassis.waitUntilDone();
    clamper.set_value(1);
    pros::delay(250);
    intake.move_velocity(-12000);
    intakefirst.move_voltage(-12000);

    chassis.turnTo(48, -20, 1000, true);
    chassis.waitUntilDone();
    rightside.move_velocity(120000);
    pros::delay(300);
    rightside.move_velocity(0);
    chassis.moveToPoint(16.3, -24, 1000, true, 80);
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.moveToPoint(16, -12, 1000, false);
    chassis.waitUntilDone();
    clamper.set_value(0);
    pros::delay(200);
    chassis.moveToPoint(19, -12, 1000, true);
    chassis.waitUntilDone();
    chassis.turnTo(22, -36, 1000, false);
    chassis.waitUntilDone();
    chassis.moveToPoint(21, -28.5, 3000, false, 60);
    intake.move_velocity(0);
    intakefirst.move_voltage(0);
    chassis.waitUntilDone();
    pros::delay(300);
    clamper.set_value(1);
    pros::delay(400);
    chassis.moveToPoint(0, -27, 1000, true, 95);
    rightside.move_voltage(12000);
    pros::delay(1000);
    rightside.move_voltage(0);
}



void bluealliancetwostakestop() {
    chassis.setPose(0, 0, 0);
    rightside.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveToPoint(0, 3.1, 1000);
    chassis.waitUntilDone();
    rightside.move_voltage(-12000);
    pros::delay(750);
    rightside.move_voltage(0);
    pros::delay(100);
    chassis.moveToPoint(0, -3, 1000, false);
    chassis.turnTo(6, 0, 1000);
    rightside.tare_position();
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(4.4, -21, 2700, false, 47);
    chassis.waitUntilDone();
    clamper.set_value(1);
    pros::delay(250);
    intake.move_velocity(-12000);
    intakefirst.move_voltage(-12000);

    chassis.turnTo(48, -20, 1500, true, 70);
    chassis.waitUntilDone();
    rightside.move_velocity(120000);
    pros::delay(300);
    rightside.move_velocity(0);
    chassis.moveToPoint(17.3, -24, 1000, true, 80);
    chassis.waitUntilDone();
    pros::delay(400);
    chassis.moveToPoint(6, -12, 1000, false);
    chassis.waitUntilDone();
    clamper.set_value(0);
    pros::delay(200);
    rightside.move_voltage(12000);
    pros::delay(1000);
    rightside.move_voltage(0);
}

void skillsRoute() {
     rightside.tare_position();
     rightside.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
     chassis.setPose(0,0,0);
     intake.move_voltage(-12000);
    intakefirst.move_voltage(-12000);
    pros::delay(500);
    intake.move_velocity(0);
    intakefirst.move_velocity(0);
    chassis.moveToPoint(0,6,1000);
    chassis.waitUntilDone();
    chassis.turnTo(24, 1.5, 900, false);
    chassis.moveToPoint(17, 1.5, 1400, false, 45);
    chassis.waitUntilDone();
    clamper.set_value(1);
    pros::delay(400);
         intake.move_voltage(-12000);
    intakefirst.move_voltage(-12000);
    chassis.turnTo(18, 24, 800);
    chassis.moveToPoint(18, 13, 1000);
    chassis.waitUntilDone();
    chassis.turnTo(32, 22, 1000, true);
    chassis.moveToPoint(29, 23, 1000);
    chassis.waitUntilDone();
    chassis.turnTo(29, 45, 1000);
    moveLiftToPosition(-297, 12000);
    chassis.moveToPoint(29, 39, 1000); 
    chassis.waitUntilDone();
    pros::delay(400);
    chassis.moveToPoint(27, 17, 1000, false);
    chassis.waitUntilDone();
    intake.move_velocity(0);
    intakefirst.move_velocity(0);
    chassis.turnTo(99, 17, 1000);
    chassis.waitUntilDone();
    pros::delay(100);
    intake.move_velocity(500);
    intakefirst.move_velocity(500);
    pros::delay(105);
    intake.move_velocity(0);
    intakefirst.move_velocity(0);
    rightside.move_voltage(-12000);
    pros::delay(250);
    rightside.move_voltage(0);
    chassis.waitUntilDone();
    intakefirst.move_velocity(-12000);
    intake.move_velocity(-12000);
    chassis.moveToPoint(35.5, 10.95, 2400, true, 60);
    chassis.waitUntilDone();
    chassis.turnTo(99, 10.95, 1000);
    chassis.waitUntilDone();
    rightside.move_velocity(-12000);
    pros::delay(1300);
    rightside.move_velocity(0);
    pros::delay(200);
    rightside.move_velocity(120000);
    pros::delay(1000);
    rightside.move_velocity(0);
    intake.move_voltage(-12000);
    intakefirst.move_voltage(-12000);
    chassis.moveToPoint(27, 14.7, 1000, false);
    chassis.turnTo(23, -10, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(22, -10.5, 5000, true, 60);
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.moveToPoint(22, -16.5, 5000, true, 60);
    chassis.waitUntilDone();
    pros::delay(200);
    chassis.turnTo(31, 4, 800);
    chassis.moveToPoint(31, 0, 1000);
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(30, -12, 1000, false);
chassis.waitUntilDone();
pros::delay(100);
clamper.set_value(0);
intake.move_velocity(12000);
intakefirst.move_velocity(12000);
pros::delay(200);
intake.move_velocity(0);
intakefirst.move_velocity(0);
chassis.moveToPoint(24, 5, 1000);
chassis.waitUntilDone();
chassis.turnTo(23, 100, 1000, false );
chassis.waitUntilDone();
chassis.moveToPoint(23, 32, 3000, false);
chassis.waitUntilDone();
chassis.turnTo(16, 45, 1000, false);
chassis.waitUntilDone();
chassis.moveToPoint(17, 46, 3000, false, 60);
chassis.waitUntilDone();
pros::delay(250);
clamper.set_value(1);
pros::delay(250);
chassis.moveToPoint(29, 28, 1500, false);
chassis.waitUntilDone();
chassis.turnTo(29, 78, 1000, true);
chassis.waitUntilDone();
hang.set_value(1);
chassis.moveToPoint(29, 48, 1000);
chassis.waitUntilDone();
chassis.turnTo(0, 48, 1000);
chassis.waitUntilDone();
chassis.moveToPoint(32, 48, 1000, false);
chassis.waitUntilDone();
clamper.set_value(0);
chassis.moveToPoint(24, 30, 1000);
chassis.waitUntilDone();
chassis.turnTo(-20, 49, 1000, false);
chassis.waitUntilDone();
chassis.moveToPoint(0, 49, 3000, false, 60);
chassis.waitUntilDone();
clamper.set_value(1);


// chassis.moveToPoint(24, 6, 1000);
// chassis.waitUntilDone();
//  intake.move_voltage(-12000);
//     intakefirst.move_voltage(-12000);
// chassis.turnTo(22, 100, 1000);
// chassis.waitUntilDone();
// chassis.moveToPoint(24, 5, 2300);
//  intake.move_voltage(0);
//     intakefirst.move_voltage(0);
// chassis.waitUntilDone();
// chassis.moveToPoint(24, 43, 1200);
// chassis.waitUntilDone();
// chassis.turnTo(-100, 48, 1000, false);
// chassis.waitUntilDone();
// chassis.moveToPoint(6, 47, 2000, false, 50);
// chassis.waitUntilDone();
// clamper.set_value(1);
// pros::delay(250);
// chassis.turnTo(50, 50, 1000);
// chassis.waitUntilDone();
}


bool runIntake = true;
void hueUpdateTask() {
    if (runIntake) {
        while (true) {
            double hue = colorsort.get_hue();
            if (hue >= 200 && hue <= 240) {
                    runIntake = false;
                pros::c::task_delay(70);
                    intake.move_voltage(12000);
                intakefirst.move_voltage(12000);
                pros::c::task_delay(400);
                    runIntake = true;
                } 
            else {
                    intake.move_voltage(-12000);
                    intakefirst.move_voltage(-12000);
            }
            pros::delay(10);
        }
    } else {
        intake.move_voltage(0);
        intakefirst.move_voltage(0);
    }
    
}


void colorSort() {
    while (true) {
            double hue = colorsort.get_hue();
            if (!runIntake) {
                intake.move_velocity(0);
                intakefirst.move_velocity(0);
            } else if (hue >= 0 && hue <= 30) {
                runIntake = false;
            pros::c::task_delay(140);
                intake.move_voltage(-12000);
            intakefirst.move_voltage(12000);
            pros::c::task_delay(400);
                runIntake = true;
            } else {
                intake.move_voltage(12000);
                intakefirst.move_voltage(-12000);
            }
        
            pros::delay(20);
        } 
}





void rednegative() {
    chassis.setPose(0, 0, 0);
    rightside.tare_position();
    rightside.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveToPoint(8, -23.8, 1200, false, 80);
    chassis.waitUntilDone();
    clamper.set_value(1);
    pros::delay(270);
    chassis.turnTo(17.5, -40, 1000, true, 80);
    chassis.waitUntilDone();
    intake.move_voltage(-12000);
                    intakefirst.move_voltage(-12000);
    chassis.moveToPoint(17.5, -35.8, 1000);
    chassis.waitUntilDone();
    chassis.turnTo(21, -35, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(24, -35, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(8.8, -24, 1000, false);
    chassis.waitUntilDone();
    chassis.turnTo(24, -23, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(15, -22, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(3, -2, 1000, true);
    chassis.waitUntilDone();
    chassis.turnTo(-12, -0.4, 1000);
    moveLiftToPosition(-300, 12000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-8,-0.4, 3000);
    chassis.waitUntilDone();
    runIntake = false;
    intake.move_velocity(0);
    intakefirst.move_velocity(0);
    ladyBrownUp(1000);
    chassis.moveToPoint(0, -6, 1000, false);
    rightside.move_velocity(12000);

    
    // ladyBrownUp(1000);
    // rightside.move_voltage(-12000);
    // pros::delay(550);
    // rightside.move_voltage(0);
    // pros::delay(100);
    // chassis.moveToPoint(1, -3, 1000, false);
    // chassis.turnTo(6, 1, 1000);
    // chassis.waitUntilDone();
    // chassis.setPose(0, 0, 0);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(2.8, -17.2, 2200, false, 60);
    // chassis.waitUntilDone();
    // clamper.set_value(1);
    // pros::delay(200);
    // pros::Task colorSortIntake(hueUpdateTask);
    // chassis.turnTo(24, -46, 1000, true, 100);
    // rightside.move_velocity(12000);
    // pros::delay(1000);
    // rightside.move_velocity(0);
    // chassis.moveToPoint(11.2, -32.9, 1000, true);
    // rightside.tare_position();
    // chassis.waitUntilDone();
    // chassis.turnTo(20, -31.9, 800);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(21.4, -31.9, 1000);
    // chassis.waitUntilDone();
    // pros::delay(300);
    // chassis.moveToPoint(1, -21, 1000, false);
    // chassis.turnTo(24, -21, 1000);
    // chassis.moveToPoint(8.7, -21, 1000);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(-12,1,1000);
    // chassis.waitUntilDone();
    // chassis.turnTo(-48, 1, 1000);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(-24, -2, 5000, 40);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(-12, -12, 1000);
}


void autonomous() {
rednegative();
}

/**
 * Runs in driver control
 */
void opcontrol() {
    rightside.tare_position();
    intake.set_reversed(true);

    bool colorSortRed = false;
    int count = 0;
    bool runIntake = true;
    colorsort.set_led_pwm(50);

    double curr_l = leftside.get_position();
    double curr_r = rightside.get_position();
    double avg_pos = (curr_l + curr_r) / 2.0;
    static bool leftwingtoggle {false};
    static bool intaketoggle {false};
    static bool hangToggle {false};
    // controller
    // loop to continuously update motors
    while (true) {
        pros::lcd::set_text(3, std::to_string(rotationalSensor.get_angle()));
        bool buttonPressed = false;
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // if (controller.get_analog(pros::E_CONTROLLER_DIGITAL_R1))

        // move the chassis with curvature drive
        chassis.tank(leftY, rightX);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) { 

    rightside.tare_position();
            
         }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            if (!leftwingtoggle) {
                clamper.set_value(1);
                leftwingtoggle = !leftwingtoggle;
            } else {
                clamper.set_value(0);
                leftwingtoggle = !leftwingtoggle;
            }
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        count = 0;
        moveLiftToPosition(-313, 6000);
        }

         if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        moveLiftToPosition(-502, 12000);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) { moveLiftToPosition(0, 12000); }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            if (!hangToggle) {
                hang.set_value(1);
                hangToggle = !hangToggle;
            } else {
                hang.set_value(0);
                hangToggle = !hangToggle;
            }
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            double hue = colorsort.get_hue();
            if (!runIntake) {
                intake.move_velocity(0);
                intakefirst.move_velocity(0);
            } else if (colorSortRed && hue >= 0 && hue <= 30 || !colorSortRed && hue >= 200 && hue <= 240) {
                runIntake = false;
            pros::c::task_delay(140);
                intake.move_voltage(-12000);
            intakefirst.move_voltage(12000);
            pros::c::task_delay(400);
                runIntake = true;
            } else {
                intake.move_voltage(12000);
                intakefirst.move_voltage(-12000);
            }
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move_voltage(-12000);
            intakefirst.move_voltage(12000);
        } else {
            intake.move_velocity(0);
            intakefirst.move_voltage(0);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            rightside.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            rightside.move_voltage(12000);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            
                if (count == 1) {
                    intakefirst.move_velocity(500);
                    pros::delay(206);
                    intakefirst.move_velocity(0);
                }
                count += 1;
                rightside.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                rightside.move_voltage(-12000);
            
           
        } else {
            rightside.move_velocity(0);
        }

        pros::delay(10);
    }
}
