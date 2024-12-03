#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"

//controller

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor intake (10);
pros::Motor arm (7);

pros::Rotation rotation (9);

pros::adi::DigitalOut doinker ('D');
pros::adi::DigitalOut clamp ('A');


// left motor group
pros::MotorGroup left_motor_group({6, -8, -20}, pros::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({2, -3, 5}, pros::MotorGears::blue);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              12, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              5 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(6);

// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              5, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(4, // proportional gain (kP)
                                              0, // integral gain (kI)
                                            20, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control

lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019// expo curve gain
);





// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors
);

int selectedAuton = 1;  
int numAutons = 3;

void auton_selector() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {

        selectedAuton = selectedAuton + 1;
        if (selectedAuton > numAutons) {
            selectedAuton = 1;
        }

        pros::delay(500);

    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {

        selectedAuton = selectedAuton - 1;
        if (selectedAuton <= 0) {
            selectedAuton = numAutons;
        }

        pros::delay(500);

    }

    controller.print(0, 0, "Auton: %d", selectedAuton);
}
// initialize function. Runs on program startup
int arm_state = 1;
bool enable_pid = false;
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
    
    pros::Task arm_task([&]() {
        int angle = 0;
        int kp = 3; 
        int maxvolt = 80;
        while (true) {
            if (arm_state == 0) {
                angle = 230;
            }
            else if (arm_state == 1) {
                angle = 195;
            }
            else if (arm_state == 2) {
                angle = 92;
            }

            int error = angle - rotation.get_angle() / 100;

            if (fabs(error) > 2 && enable_pid) {
                int speed = kp * error;
                if (speed < -maxvolt) {
                    speed = -maxvolt;
                }
                if (speed > maxvolt) {
                    speed = maxvolt;
                }
                arm.move(speed);
    
            } 
            else {
                arm.brake();
                enable_pid = false;
            }

            pros::delay(20);
        }

    });
    
}

void disabled() {
    clamp.set_value(true);
}

void competition_initialize() {
    clamp.set_value(true);
}

void auton1() {
    chassis.setPose(0, 0, 0);
    //chassis.turnToHeading(90, 2000);
    //chassis.moveToPoint(20, 0, 2000);
    chassis.moveToPose(24, 24, 90, 2000);
    chassis.moveToPose(0, 0, 90, 2000);
    chassis.moveToPose(24, 24, 270, 2000);
    chassis.moveToPose(0, 0, 0, 2000);
    /*
    chassis.setPose(0, 0, 0);
    clamp.set_value(true);
    chassis.moveToPoint(0, -24, 2000, {.forwards = false, .maxSpeed = 50});
    clamp.set_value(false);
    pros::delay(400);
    chassis.moveToPoint(-20, -28, 2000, {.maxSpeed = 80});
    intake.move(127);
    intake2.move(-127);
    pros::delay(1000);
    chassis.turnToPoint(-13, -47, 2000);
    chassis.moveToPoint(-13, -47, 2000, {.maxSpeed = 70});
    pros::delay(1000);
    chassis.moveToPoint(-13, -35, 2000, {.forwards = false});
    chassis.moveToPoint(-18, -47, 2000, {.maxSpeed = 70});
    pros::delay(1500);
    chassis.moveToPoint(5, -30, 2000, {.forwards = false});
    chassis.turnToPoint(20, -46, 2000);
    intake.move_velocity(0);
    intake2.move_velocity(0);
    clamp.set_value(true);
    chassis.moveToPoint(20, -46, 2000, {.maxSpeed = 50});
    */
}

void auton2() {
    arm_state = 1;
    pros::delay(1000);
    arm_state = 0;
    /*
    chassis.setPose(0, 0, 0);
    clamp.set_value(true);
    chassis.moveToPoint(0, -24, 2000, {.forwards = false, .maxSpeed = 50});
    clamp.set_value(false);
    pros::delay(400);
    intake.move(127);
    intake2.move(-127);
    chassis.moveToPoint(20, -25, 2000, {.maxSpeed = 80});
    pros::delay(800);
    chassis.turnToPoint(36, -5, 2000, {.forwards = false});
    chassis.moveToPoint(36, -5, 2000, {.forwards = false});
    intake.move(0);
    intake2.move(0);
    clamp.set_value(true);
    chassis.turnToPoint(30, -40, 2000, {.forwards = false});
    chassis.moveToPoint(30, -40, 2000, {.forwards = false});
    chassis.moveToPoint(25, -52, 2000, {.forwards = false, .maxSpeed = 50});
    clamp.set_value(false);
    pros::delay(400);
    chassis.moveToPoint(0, -30, 2000);
    chassis.turnToPoint(-15, -48, 2000);
    chassis.moveToPoint(-15, -48, 2000, {.maxSpeed = 50});
    */
}

void autonomous() {
        switch (selectedAuton) {
        case 1:
            auton1();
            break;
        case 2:
            auton2();
            break;
    }
}

void intake_control() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        intake.move(127);
    }
    
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        intake.move(-127);
    }   
    
    else {
        intake.move_velocity(0);
    }
}


bool toggle = false; 
bool latch = false;

void clamp_control() {
    if (toggle) {
        clamp.set_value(false);
    } 
    else {
        clamp.set_value(true);
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        if(!latch){ 
            toggle = !toggle;
            latch = true;
        }
    } 
    else {
        latch = false; 
    }
}

bool toggle2 = false;
bool latch2 = false;

void doinker_control() {
    if (toggle2) {
        doinker.set_value(true);
    } 
    else {
        doinker.set_value(false);
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        if(!latch2) { 
            toggle2 = !toggle2;
            latch2 = true;
        }
    } 
    else {
        latch2 = false; 
    }
}

bool button_pressed = false;
void arm_control() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        if (!button_pressed) {
            arm_state = (arm_state + 1) % 3;
            enable_pid = true;
            button_pressed = true;
        }
    }
    else {
        button_pressed = false;
    }
}

void opcontrol() {
    // loop forever
    while (true) {
        auton_selector();
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);
        intake_control();
        doinker_control();
        clamp_control();
        arm_control();

        // delay to save resources
        pros::delay(25);
    }
}