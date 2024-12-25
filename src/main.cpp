#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
/*
_______________      _____ ________   ____ 
\_____  \   _  \    /  |  /_   \   \ /   / 
 /  ____/  /_\  \  /   |  ||   |\   Y   /  
/       \  \_/   \/    ^   /   | \     /   
\_______ \_____  /\____   ||___|  \___/    
        \/     \/      |__|  
*/              

//controller

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor intake (5);
pros::Motor arm (2);
pros::Motor arm2 (10);
pros::Rotation rotation (11);
pros::Optical optical(18);

pros::Rotation horizontal_encoder(20);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -3.25);

pros::adi::DigitalOut doinker ('A');
pros::adi::DigitalOut clamp ('H');


// left motor group
pros::MotorGroup left_motor_group({4, -3, -6}, pros::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({7, -9, 8}, pros::MotorGears::blue);

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
int numAutons = 5;
std::string team = "red";
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
        name = "red 4ring";
        team = "red";
    }

    else if (selectedAuton == 2) {
        name = "red rush";
        team = "red";
    }

    else if (selectedAuton == 3) {
        name = "blue 4ring";
        team = "blue";
    }

    else if (selectedAuton == 4) {
        name = "blue rush";
        team = "blue";
    }

    else if (selectedAuton == 5) {
        name = "skills";
        team = "red";
    }
    controller.print(0, 0, "Auton: %d %s", selectedAuton, name);
}
// initialize function. Runs on program startup
int arm_state = 0;
bool enable_pid = false;
int colorSort = false;
int intakeTime = 0;
std::string intakeDirection = "forward";
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    optical.set_integration_time(3);
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
    
    pros::Task intake_task([&]() { 
        while (true) {
            if (intake.get_actual_velocity() >= 20) {
                optical.set_led_pwm(100);
                double hue = optical.get_hue();
                if (team == "red" && hue > 200 && hue < 250) {
                    colorSort = true;
                    pros::delay(95);
                    intake.move(-127);
                    pros::delay(300);
                    colorSort = false;
                }
                if (team == "blue" && hue > 0 && hue < 30) {
                    colorSort = true;
                    pros::delay(95);
                    intake.move(-127);
                    pros::delay(300);
                    colorSort = false;
                }
            }
            else {
                optical.set_led_pwm(0);
            }
            pros::delay(10);
            
        }
    });

    pros::Task intake_toggle([&]() { 
        while (true) {
            if (pros::competition::is_autonomous()) {
                if (intakeTime > 0) {
                    if (intakeDirection == "forward" && !colorSort) {
                        intake.move(127);
                    }
                    else if (intakeDirection == "backward" && !colorSort) {
                        intake.move(-127);
                    }
                    intakeTime -= 10;
                }
                else {
                    intake.move(0);
                }
            }
            pros::delay(10);
            
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

//red 4ring
void auton1() {
    chassis.setPose(0, 0, 0);
    intakeTime = 10000;
    intakeDirection = "forward";
    //chassis.moveToPose(24, 24, 50, 2000);
    /*
    clamp.set_value(true);
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, -13, 2000, {.forwards = false});
    chassis.turnToPoint(12, -26, 2000, {.forwards = false});
    chassis.moveToPoint(12, -26, 2000, {.forwards = false, .maxSpeed = 50});
    pros::delay(1300);
    clamp.set_value(false);
    intake.move(127);
    pros::delay(1100);
    chassis.turnToPoint(20, -20, 2000);
    chassis.moveToPoint(20, -20, 2000);
    pros::delay(1500);
    chassis.moveToPoint(17, -15, 2000, {.forwards = false});
    chassis.turnToPoint(24, -30.5, 2000);
    chassis.moveToPoint(24, -30.5, 2000, {.maxSpeed = 60});
    pros::delay(1200);
    chassis.moveToPoint(34, -18, 2000, {.forwards = false});
    chassis.turnToPoint(34, -30.5, 2000);
    chassis.moveToPoint(34, -30.5, 2000, {.maxSpeed = 60});
    pros::delay(1200);
    chassis.moveToPoint(27, -9, 2000, {.forwards = false});
    chassis.turnToPoint(11, -25, 2000);
    arm_state = 2;
    enable_pid = true;
    chassis.moveToPoint(11, -25, 2000, {.maxSpeed = 80});
    intake.move(0);
    */
}

//red rush
void auton2() {
    clamp.set_value(true);
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, -36.5, 2000, {.forwards = false});
    chassis.turnToHeading(327, 2000);
    chassis.moveToPoint(8.9, -44.5, 2000, {.forwards = false, .maxSpeed = 25});
    pros::delay(1700);
    clamp.set_value(false);
    intake.move(127);
    pros::delay(1500);
    chassis.turnToPoint(7, -29, 2000);
    chassis.moveToPoint(7, -29, 2000);
    pros::delay(2300);
    intake.move(0);
    


}

//blue 4ring
void auton3() {
    clamp.set_value(true);
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, -13, 2000, {.forwards = false});
    chassis.turnToPoint(-13, -26, 2000, {.forwards = false});
    chassis.moveToPoint(-13, -26, 2000, {.forwards = false, .maxSpeed = 50});
    pros::delay(1300);
    clamp.set_value(false);
    intake.move(127);
    pros::delay(1100);
    chassis.turnToPoint(-27, -26, 2000);
    chassis.moveToPoint(-27, -26, 2000);
    pros::delay(1500);
    chassis.moveToPoint(-26, -19, 2000, {.forwards = false});
    chassis.turnToPoint(-29, -34, 2000);
    chassis.moveToPoint(-29, -34, 2000, {.maxSpeed = 60});
    pros::delay(1200);
    chassis.moveToPoint(-39, -18, 2000, {.forwards = false});
    chassis.turnToPoint(-39, -34, 2000);
    chassis.moveToPoint(-39, -34, 2000, {.maxSpeed = 60});
    pros::delay(1200);
    chassis.moveToPoint(-27, -9, 2000, {.forwards = false});
    chassis.turnToPoint(-11, -25, 2000);
    arm_state = 2;
    enable_pid = true;
    chassis.moveToPoint(-9, -25, 2000, {.maxSpeed = 80});
    intake.move(0);

}

//blue rush
void auton4() {
    clamp.set_value(true);
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, -35, 2000, {.forwards = false});
    chassis.turnToHeading(30, 2000);
    chassis.moveToPoint(-10.2, -42.3, 2000, {.forwards = false, .maxSpeed = 25});
    pros::delay(1700);
    clamp.set_value(false);
    intake.move(127);
    pros::delay(1500);
    chassis.turnToPoint(-9, -29, 2000);
    chassis.moveToPoint(-9, -29, 2000);
    pros::delay(2000);
    chassis.turnToHeading(180, 2000);
    pros::delay(2300);
    intake.move(0);
}

//skills

void auton5() {
    clamp.set_value(true);
    chassis.setPose(-58.7, 0, 90);
    intake.move(127);
    pros::delay(1800);
    intake.move(-127);
    pros::delay(400);
    intake.move(0);
    chassis.moveToPoint(-52, 0, 2000);
    chassis.turnToPoint(-46.8, 16, 2000, {.forwards = false});
    chassis.moveToPoint(-46.8, 16, 2000, {.forwards = false, .maxSpeed = 40});
    pros::delay(1500);
    clamp.set_value(false);
    chassis.turnToPoint(-22.5, 23.5, 2000, {.maxSpeed = 60});
    intake.move(127);
    chassis.moveToPoint(-22.5, 23.5, 2000, {.maxSpeed = 60});
    pros::delay(800);
    chassis.turnToPoint(-23, 47, 2000, {.maxSpeed = 60});
    chassis.moveToPoint(-23, 47, 2000, {.maxSpeed = 60});
    pros::delay(800);
    chassis.turnToPoint(-23, 47, 2000, {.maxSpeed = 60});
    chassis.moveToPoint(-23, 47, 2000, {.maxSpeed = 60});
    pros::delay(800);
    chassis.turnToPoint(-60, 46, 2000, {.maxSpeed = 60});
    chassis.moveToPoint(-60, 46, 2000, {.maxSpeed = 60});
    pros::delay(800);
    chassis.turnToPoint(-60, 46, 2000, {.forwards = false});
    chassis.moveToPoint(-60, 46, 2000, {.forwards = false});
    pros::delay(800);
    chassis.moveToPoint(-47.5, 46, 2000, {.forwards = false});
    pros::delay(800);
    chassis.turnToPoint(-47.5, 58.5, 2000, {.maxSpeed = 60});
    chassis.moveToPoint(-47.5, 58.5, 2000, {.maxSpeed = 60});
    chassis.moveToPoint(-47.5, 46, 2000, {.forwards = false});
    chassis.turnToPoint(-59.5, 60, 2000, {.forwards = false});
    chassis.moveToPoint(-59.5, 60, 2000, {.forwards = false});
    chassis.moveToPoint(-47.5, 46, 2000);






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
    }
}

void intake_control() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        if (!colorSort) {
            intake.move(127);
        }
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