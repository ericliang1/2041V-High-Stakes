#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"

//controller

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor intake (5);
pros::Motor arm (2);
pros::Motor arm2 (10);

pros::Rotation rotation (11);
pros::Rotation horizontal_encoder(20);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 4.5);
//pros::Optical optical (10);

pros::adi::DigitalOut doinker ('A');
pros::adi::DigitalOut clamp ('H');


// left motor group
pros::MotorGroup left_motor_group({4, -3, -6}, pros::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({7, 8, -9}, pros::MotorGears::blue);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              12, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(19);

// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(20, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              2 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(6, // proportional gain (kP)
                                              0, // integral gain (kI)
                                            45, // derivative gain (kD)
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
int numAutons = 6;
int team = 0;
void auton_selector() {
    std::string name;
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

    if (selectedAuton == 1) {
        name = "a";
        team = 0;
    }

    else if (selectedAuton == 2) {
        name = "b";
        team = 0;
    }

    else if (selectedAuton == 3) {
        name = "c";
        team = 0;
    }

    else if (selectedAuton == 4) {
        name = "d";
        team = 1;
    }

    else if (selectedAuton == 5) {
        name = "e";
        team = 1;
    }

    else if (selectedAuton == 6) {
        name = "f";
        team = 1;
    }

    controller.print(0, 0, "Auton: %d %s", selectedAuton, name);
}
// initialize function. Runs on program startup
int arm_state = 0;
bool enable_pid = false;
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    arm2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
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
        double angle = 0;
        int time_settled = 0;
        double kp = 3; 
        double kd = 0.5;
        int maxvolt = 127;
        double previous_error = 0;
        while (true) {
            if (arm_state == 0) {
                angle = 75;
            }
            else if (arm_state == 1) {
                angle = 101;
            }
            else if (arm_state == 2) {
                angle = 208;
            }

            double error = angle - rotation.get_angle() / 100.0;

            if (time_settled < 60 && enable_pid) {
                int speed = kp * error + kd * (error - previous_error);
                if (speed < -maxvolt) {
                    speed = -maxvolt;
                }
                if (speed > maxvolt) {
                    speed = maxvolt;
                }
                arm.move(-speed);
                arm2.move(speed);
                if (fabs(error) <= 0.2) {
                    time_settled += 20;
                }
                else {
                    time_settled = 0;
                }
    
            } 
            else {
                time_settled = 0;
                arm.brake();
                arm2.brake();
                enable_pid = false;
            }

            previous_error = error;

            pros::delay(10);
        }

    });

}

void disabled() {
    clamp.set_value(false);
}

void competition_initialize() {
    clamp.set_value(false);
}

//red solowp
void auton1() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 10, 2000);
    chassis.turnToHeading(90, 2000);
    chassis.moveToPoint(10, 10, 2000);
    chassis.moveToPoint(0, 0, 0);

    /*
    chassis.turnToHeading(90, 2000);
    chassis.moveToPoint(10, 10, 2000);
    chassis.moveToPose(0, 0, 0, 2000);
    */
    /*
    chassis.setPose(-54, 32, 0);
    chassis.moveToPoint(-54, -0.5, 2000, {.forwards = false});
    chassis.turnToHeading(90, 2000);
    chassis.moveToPoint(-59.5, -0.5, 2000, {.forwards = false});
    pros::delay(200);
    intake.move(127);
    pros::delay(1500);
    intake.move(0);
    chassis.moveToPoint(-52, 0, 2000);
    chassis.moveToPose(-28, 22, 232, 2000, {.forwards = false});
    pros::delay(1600);
    clamp.set_value(true);
    intake.move(127);
    chassis.moveToPoint(-23, 45, 2000);
    pros::delay(800);
    chassis.moveToPoint(-2, 50, 2000, {.maxSpeed = 50});
    pros::delay(800);
    chassis.moveToPoint(-27, 42, 2000, {.forwards = false});
    chassis.turnToPoint(-2, 42, 2000);
    chassis.moveToPoint(-2, 42, 2000, {.maxSpeed = 50});
    pros::delay(800);
    chassis.moveToPoint(-28, 33, 2000, {.forwards = false});
    chassis.moveToPose(-30, 8, 140, 2000);
    intake.move(0);
    chassis.moveToPoint(-25, 2, 2000, {.maxSpeed = 80});
    */
}

//red 4ring
void auton2() {
    chassis.setPose(-57, 7, 270);
    chassis.moveToPose(-28, 20, 232, 2000, {.forwards = false});
    pros::delay(1600);
    clamp.set_value(true);
    intake.move(127);
    chassis.moveToPoint(-23, 42, 2000);
    pros::delay(800);
    chassis.moveToPoint(-9, 47, 2000, {.maxSpeed = 50});
    pros::delay(800);
    chassis.moveToPoint(-27, 39, 2000, {.forwards = false});
    chassis.turnToPoint(-9, 39, 2000);
    chassis.moveToPoint(-9, 39, 2000, {.maxSpeed = 50});
    pros::delay(800);
    chassis.moveToPoint(-28, 33, 2000, {.forwards = false});
    chassis.moveToPose(-30, 8, 140, 2000);
    intake.move(0);
    chassis.moveToPoint(-25, 2, 2000, {.maxSpeed = 80});
}

//red positive corner
void auton3() {
    chassis.setPose(-54, -37, 270);
    chassis.moveToPose(-28, -26, 232, 2000, {.forwards = false});
    pros::delay(2000);
    clamp.set_value(true);
    pros::delay(1000);
    intake.move(127);
    chassis.moveToPoint(-23, -47, 2000);
    pros::delay(2000);
    //chassis.moveToPose(0, -20, 90, 3000, {.maxSpeed = 80});
    pros::delay(2000);
    intake.move(0);
}

//blue solowp
void auton4() {
    chassis.setPose(54, 32, 0);
    chassis.moveToPoint(54, -0.5, 2000, {.forwards = false});
    chassis.turnToHeading(273, 2000); // Adjusted heading
    chassis.moveToPoint(60, -0.5, 2000, {.forwards = false});
    pros::delay(200);
    intake.move(127);
    pros::delay(1500);
    intake.move(0);
    chassis.moveToPoint(52, 0, 2000);
    chassis.moveToPose(28, 20, 128, 2000, {.forwards = false}); 
    pros::delay(1600);
    clamp.set_value(true);
    intake.move(127);
    chassis.moveToPoint(23, 45, 2000);
    pros::delay(800);
    chassis.moveToPoint(7, 50, 2000, {.maxSpeed = 50});
    pros::delay(800);
    chassis.moveToPoint(27, 42, 2000, {.forwards = false});
    chassis.turnToPoint(7, 42, 2000);
    chassis.moveToPoint(7, 42, 2000, {.maxSpeed = 50});
    pros::delay(800);
    chassis.moveToPoint(28, 33, 2000, {.forwards = false});
    chassis.moveToPose(30, 8, 220, 2000); 
    intake.move(0);
    chassis.moveToPoint(25, -2, 2000, {.maxSpeed = 80});
}

//blue 4ring

void auton5() {
    chassis.setPose(-54, 6, 270); 
    chassis.moveToPose(-22, 21, 225, 2000, {.forwards = false}); 
    pros::delay(1600);   
    clamp.set_value(true);
    pros::delay(800);
    intake.move(127);
    chassis.moveToPoint(-24, 45, 2000);
    pros::delay(1800);
    chassis.moveToPoint(-8, 47, 2000, {.maxSpeed = 70});
    pros::delay(1000);
    chassis.moveToPoint(-20, 47, 2000, {.forwards = false});
    chassis.moveToPose(-6, 38, 90, 2500);
    pros::delay(3000);
    //chassis.moveToPoint(-20, 38, 2000, {.forwards = false});
    intake.move(0);

}

//blue positive corner

void auton6() {
    chassis.setPose(-54, 6, 270); 
    chassis.moveToPose(-22, 21, 225, 2000, {.forwards = false}); 
    pros::delay(1800);   
    clamp.set_value(true);
    pros::delay(1000);
    intake.move(127);
    chassis.moveToPoint(-24, 48, 2000);
    pros::delay(2000);
    //chassis.moveToPose(-4, 19, 100, 3000, {.maxSpeed = 80}); 
    pros::delay(2000);
    intake.move(0);

}

void autonomous() {
    switch (selectedAuton) {
        //
        case 1:
            auton1();
            break;
        case 2:
            auton2();
            break;
        
        case 3:
            auton3();
            break;

        case 4:
            auton4();
            break;

        case 5:
            auton5();
            break;

        case 6:
            auton6();
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
        clamp.set_value(true);
    } 
    else {
        clamp.set_value(false);
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

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
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
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
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