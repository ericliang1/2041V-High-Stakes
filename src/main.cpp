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

pros::Motor intake (-5);
pros::Motor arm (2);
pros::Motor arm2 (10);
pros::Rotation rotation (11);
pros::Optical optical(1);

pros::Rotation horizontal_encoder(20);
pros::Rotation vertical_encoder(18);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 2.5);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0.7);

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
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(12, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              6, // derivative gain (kD)
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
                                     1.027 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.027// expo curve gain
);





// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors
);

int selectedAuton = 1;  
int numAutons = 7;
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
        name = "red solo awp";
        team = "red";
    }

    else if (selectedAuton == 4) {
        name = "blue 4ring";
        team = "blue";
    }

    else if (selectedAuton == 5) {
        name = "blue rush";
        team = "blue";
    }

    else if (selectedAuton == 6) {
        name = "blue solo awp";
        team = "blue";
    }

    else if (selectedAuton == 7) {
        name = "forward";
        team = "nothing";
    }
    controller.print(0, 0, "Auton: %d %s", selectedAuton, name);
}
// initialize function. Runs on program startup
int arm_state = 0;
bool enable_pid = false;
int colorSort = false;
int intakeTime = 0;
bool intakeStop = false;
bool isAuton = true;
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
            if (isAuton && team == "red" || team == "blue") {
                optical.set_led_pwm(50);
                double hue = optical.get_hue();
                if (team == "red" && hue > 208 && hue < 223){
                    colorSort = true;
                    pros::delay(250);
                    intake.move(-127);
                    pros::delay(300);
                    colorSort = false;
                }
                if (team == "blue" && hue > 0 && hue < 15) {
                    colorSort = true;
                    pros::delay(250);
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
                if (intakeTime > 0 && !intakeStop) {
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
                angle = 175;
            }
            else if (arm_state == 1) {
                angle = 199;
            }
            else if (arm_state == 2) {
                angle = 310;
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

            pros::lcd::print(4, "angle: %f", rotation.get_angle() / 100.0);

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
    clamp.set_value(true);
    chassis.setPose(-57.84,35.89,270);
    chassis.moveToPoint(-44,35.89,2000,{.forwards = false});
    chassis.turnToPoint(-28, 25, 2000, {.forwards = false});
    chassis.moveToPoint(-28, 25, 2000, {.forwards = false, .maxSpeed = 80});
    pros::delay(800);
    clamp.set_value(false);
    chassis.moveToPoint(-12.2, 40, 2000, {.maxSpeed = 80});
    pros::delay(400);
    intakeDirection = "forward";
    intakeTime = 12000;
    chassis.moveToPoint(-12.2, 60, 2000, {.maxSpeed = 40});
    chassis.moveToPose(-28.5, 26, 45, 2000, {.forwards = false});
    chassis.moveToPoint(-31.5, 55, 2000, {.maxSpeed = 90});
    pros::delay(1200);
    chassis.moveToPoint(-32.5, 26, 2000, {.forwards = false});
    chassis.moveToPoint(-71, 64, 2000, {.maxSpeed = 100});
    pros::delay(3000);
    chassis.moveToPoint(-50, -10, 2000);

}

//red rush
void auton2() {
    clamp.set_value(true);
    chassis.setPose(-52, -62, 90);
    chassis.moveToPoint(-27, -62, 2000, {.minSpeed = 127, .earlyExitRange = 5});
    chassis.moveToPoint(-17, -58, 2000, {.minSpeed = 60});
    pros::delay(700);
    doinker.set_value(true);
    pros::delay(200);
    chassis.moveToPoint(-31.5, -62, 2000, {.forwards = false});
    pros::delay(1300);
    doinker.set_value(false);
    pros::delay(300);
    chassis.turnToPoint(-17.5, -67, 2000, {.forwards = false});
    chassis.moveToPoint(-17.5, -67, 2000, {.forwards = false, .maxSpeed = 60});
    pros::delay(1300);
    clamp.set_value(false);
    chassis.moveToPoint(-35, -50, 2000, {.maxSpeed = 80});
    intakeDirection = "forward";
    intakeTime = 1300;
    pros::delay(1700);
    clamp.set_value(true);
    chassis.turnToPoint(-29, -27.5, 2000, {.forwards = false});
    chassis.moveToPoint(-29, -27.5, 2000, {.forwards = false, .maxSpeed = 60});
    pros::delay(1300);
    clamp.set_value(false);
    pros::delay(300);
    chassis.moveToPoint(-38, -55, 2500);
    pros::delay(200);
    intakeTime = 2000;
    pros::delay(1000);
    clamp.set_value(true);
    pros::delay(300);
    chassis.turnToHeading(270, 2000);
}

//blue 4ring
void auton3() {
    clamp.set_value(true);
    chassis.setPose(-56, 20, 0);
    chassis.moveToPoint(-56, 2, 2000, {.forwards = false});
    chassis.turnToHeading(270, 2000);
    chassis.moveToPoint(-63, 0.5, 2000, {.minSpeed = 20});
    pros::delay(850);
    arm_state = 2;
    enable_pid = true;
    pros::delay(1000);
    arm_state = 0;
    enable_pid = true;
    pros::delay(200);
    chassis.moveToPoint(-22, 24, 2000, {.forwards = false, .maxSpeed = 70});
    pros::delay(1800);
    clamp.set_value(false);
    pros::delay(300);
    chassis.moveToPoint(-24.5, 49, 2000, {.maxSpeed = 70});
    intakeDirection = "forward";
    intakeTime = 15000;
    chassis.moveToPoint(-24.5, 37, 2000, {.forwards = false});
    chassis.turnToPoint(-12.5, 40, 2000);
    chassis.moveToPoint(-12.5, 40, 2000, {.maxSpeed = 80});
    chassis.moveToPoint(-23.4, 55, 2000, {.forwards = false});
    chassis.turnToPoint(-12, 57, 2000);
    chassis.moveToPoint(-12, 57, 2000, {.maxSpeed = 80});
    chassis.moveToPoint(-34.6, 33.2, 2000, {.forwards = false});
    arm_state = 2;
    enable_pid = true;
    chassis.moveToPoint(-19, 19, 2000);


}

//blue rush
void auton4() {
    clamp.set_value(true);
    chassis.setPose(-57.84,-35.89,270);
    chassis.moveToPoint(-44,-35.89,2000,{.forwards = false});
    chassis.turnToPoint(-26, -25, 2000, {.forwards = false});
    chassis.moveToPoint(-26, -25, 2000, {.forwards = false, .maxSpeed = 80});
    pros::delay(800);
    clamp.set_value(false);
    chassis.moveToPoint(-7.5, -42, 2000, {.maxSpeed = 80});
    pros::delay(400);
    intakeDirection = "forward";
    intakeTime = 12000;
    chassis.moveToPoint(-5, -60, 2000, {.maxSpeed = 60});
    chassis.moveToPose(-26.5, -24, 115, 2000, {.forwards = false});
    chassis.moveToPoint(-25.5, -50, 2000, {.maxSpeed = 90});
    pros::delay(1200);
    chassis.moveToPoint(-25.5, -33, 2000, {.forwards = false});
    chassis.moveToPoint(-64, -68, 2000, {.maxSpeed = 100});
    pros::delay(3000);
    chassis.moveToPoint(-54, -58, 2000, {.forwards = false, .minSpeed = 40});
    chassis.moveToPoint(-64, -68, 2000, {.minSpeed = 100});
}

//skills

void auton5() {



    
    clamp.set_value(true);
    chassis.setPose(-52, 62, 90);
    chassis.moveToPoint(-24.5, 62, 2000, {.minSpeed = 127, .earlyExitRange = 5});
    chassis.moveToPoint(-17, 61, 2000, {.minSpeed = 60});
    pros::delay(700);
    doinker.set_value(true);
    pros::delay(200);
    chassis.moveToPose(-32, 65, 90, 2000, {.forwards = false, .minSpeed = 10});
    pros::delay(1000);
    doinker.set_value(false);
    pros::delay(300);
    chassis.turnToPoint(-11.5, 58.3, 2000, {.forwards = false});
    chassis.moveToPoint(-11.5, 58.3, 2000, {.forwards = false, .maxSpeed = 60});
    pros::delay(1300);
    clamp.set_value(false);
    chassis.moveToPoint(-27, 46, 2000, {.maxSpeed = 80});
    intakeDirection = "forward";
    intakeTime = 900;
    pros::delay(1700);
    clamp.set_value(true);
    chassis.turnToPoint(-31, 19, 2000, {.forwards = false});
    chassis.moveToPoint(-31, 19, 2000, {.forwards = false, .maxSpeed = 60});
    pros::delay(1300);
    clamp.set_value(false);
    pros::delay(300);
    chassis.moveToPoint(-45, 49, 2500);
    pros::delay(200);
    intakeTime = 2000;
    pros::delay(1000);
    clamp.set_value(true);
    pros::delay(300);
    chassis.turnToHeading(270, 2000);
}
/*

void auton5() {
    clamp.set_value(true);
    chassis.setPose(-59, 0, 90);
    intakeDirection = "forward";
    intakeTime = 800;
    pros::delay(600);
    chassis.moveToPoint(-45, 0, 2000);
    chassis.turnToPoint(-47, -19.5, 2000, {.forwards = false});
    chassis.moveToPoint(-47, -19.5, 2000, {.forwards = false, .maxSpeed = 60});
    pros::delay(1200);
    clamp.set_value(false);
    pros::delay(300);
    intakeTime = 10000;
    chassis.moveToPoint(-25, -20.5, 2000);
    pros::delay(1800);
    chassis.turnToPoint(1.5, -55, 2000);
    chassis.moveToPoint(1.5, -55, 2000, {.maxSpeed = 70});
    arm_state = 1;
    //enable_pid = true;
    chassis.turnToHeading(180, 2000);
    chassis.moveToPoint(1.5, -60, 2000);
    pros::delay(1500);
    intakeStop = true;
    arm_state = 2;
    //enable_pid = true;
    pros::delay(1000);
    arm_state = 0;
    enable_pid = true;
    chassis.moveToPoint(0, -49, 2000, {.forwards = false});
    chassis.turnToPoint(-56.5, -47.5, 2000);
    intakeStop = false;
    intakeTime = 10000;
    chassis.moveToPoint(-56.5, -47.5, 4000, {.maxSpeed = 50});
    pros::delay(3400);
    chassis.turnToPoint(-44, -61, 2000);
    chassis.moveToPoint(-44, -61, 2000);
    pros::delay(2300);
    chassis.turnToPoint(-50, -62, 2000, {.forwards = false});
    chassis.moveToPoint(-50, -62, 800, {.forwards = false});
    clamp.set_value(true);
    intakeStop = true;
    chassis.moveToPoint(-39, -47, 2000);
    chassis.turnToHeading(180, 2000);
    chassis.moveToPoint(-35.5, 9, 2000, {.forwards = false});
    chassis.moveToPoint(-35.5, 22, 2000, {.forwards = false, .maxSpeed = 60});
    pros::delay(1500);
    clamp.set_value(false);
    pros::delay(300);
    intakeStop = false;
    intakeTime = 20000;
    chassis.moveToPoint(-13.5, 26.5, 2000);
    pros::delay(1500);
    chassis.turnToPoint(-11.5, 51.5, 2000);
    chassis.moveToPoint(-11.5, 51.5, 2000);
    pros::delay(1500);
    chassis.turnToPoint(-53, 50, 2000);
    chassis.moveToPoint(-53, 50, 2000, {.maxSpeed = 50});
    pros::delay(2900);
    chassis.turnToPoint(-42, 61, 2000);
    chassis.moveToPoint(-42, 61, 2000);
    pros::delay(2300);
    chassis.turnToPoint(-48, 64, 2000, {.forwards = false});
    chassis.moveToPoint(-48, 64, 800, {.forwards = false});
    clamp.set_value(true);
    chassis.moveToPoint(-7.5, 61.5, 2000);
    arm_state = 1;
    enable_pid = true;
    chassis.moveToPoint(7.5, 61.5, 2000, {.maxSpeed = 80});
    pros::delay(2000);
    chassis.turnToHeading(0, 2000);
    chassis.moveToPoint(6, 65, 2000);
    pros::delay(1000);
    intakeStop = true;
    arm_state = 2;
    enable_pid = true;
    pros::delay(1000);
    arm_state = 0;
    enable_pid = true;
    */
    /*
    chassis.setPose(0, 62, 0);
    chassis.moveToPoint(0, 51, 2000, {.forwards = false});
    intakeStop = false;
    intakeTime = 1500;
    chassis.turnToPoint(34.6, 10.2, 2000);
    chassis.turnToPoint(52.4, 19.3, 2000, {.forwards = false});
    chassis.moveToPoint(52.4, 19.3, 2000, {.forwards = false, .maxSpeed = 60});
    pros::delay(1000);
    clamp.set_value(true);
    pros::delay(300);
    intakeTime = 4000;
    chassis.moveToPoint(57.9, 42, 2000);
    pros::delay(400);
    chassis.turnToPoint(60.9, 54.4, 2000, {.forwards = false});
    chassis.moveToPoint(60.9, 54.4, 800, {.forwards = false});
    clamp.set_value(true);
    */








    /*
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
    */






//}

void auton6() {
    clamp.set_value(true);
    chassis.setPose(56, 20, 0);
    chassis.moveToPoint(56, 1.5, 2000, {.forwards = false});
    chassis.turnToHeading(90, 2000);
    chassis.moveToPoint(64, 4, 2000, {.minSpeed = 30});
    pros::delay(850);
    arm_state = 2;
    enable_pid = true;
    pros::delay(1000);
    arm_state = 0;
    enable_pid = true;
    chassis.moveToPoint(21, 32, 2000, {.forwards = false, .maxSpeed = 70});
    pros::delay(1900);
    clamp.set_value(false);
    pros::delay(300);
    chassis.moveToPoint(23, 55, 2000, {.maxSpeed = 80});
    intakeDirection = "forward";
    intakeTime = 15000;
    chassis.moveToPoint(25, 45, 2000, {.forwards = false, .maxSpeed = 80});
    chassis.turnToPoint(7.5, 47, 2000);
    chassis.moveToPoint(7.5, 47, 2000, {.maxSpeed = 80});
    chassis.moveToPoint(23.4, 55, 2000, {.forwards = false});
    chassis.turnToPoint(7.5, 55, 2000);
    chassis.moveToPoint(7.5, 55, 2000, {.maxSpeed = 80});
    chassis.moveToPoint(34.6, 33.2, 2000, {.forwards = false});
    arm_state = 2;
    enable_pid = true;
    chassis.moveToPoint(18.5, 18.5, 2000);//0
}

void auton7() {
    clamp.set_value(true);
    chassis.setPose(-61, 0, 90);
    intakeDirection = "forward";
    intakeTime = 800;
    pros::delay(600);
    chassis.moveToPoint(-49, 0, 2000);
    chassis.turnToHeading(0, 2000);
    chassis.moveToPoint(-49.5, -19.5, 2000, {.forwards = false, .maxSpeed = 80});
    pros::delay(700);
    clamp.set_value(false);
    intakeTime = 10000;
    chassis.moveToPoint(-23.5, -23.5, 2000);
    //chassis.moveToPoint(-3.5, -36, 2000);
    chassis.moveToPoint(23.5, -46, 2000);
    chassis.moveToPoint(-3.5, -37.5, 2000, {.forwards = false});
    chassis.turnToHeading(180, 2000);
    intakeStop = true;
    arm_state = 3;
    enable_pid = true;
    intakeStop = false;
    intakeTime = 10000;
    chassis.moveToPoint(-3.8, -56, 2000, {.maxSpeed = 60});
    intakeStop = true;
    arm_state = 2;
    enable_pid = true;
    pros::delay(600);
    arm_state = 0;
    enable_pid = true;
    pros::delay(400);
    chassis.moveToPoint(0, -40, 2000, {.forwards = false});
    chassis.turnToPoint(-23.5, -40, 2000);
    intakeStop = false;
    intakeTime = 10000;
    chassis.moveToPoint(-23.5, -40, 2000);
    chassis.moveToPoint(-63, -39, 2000,{.maxSpeed = 35});
    pros::delay(2000);
    chassis.turnToPoint(-43, -55, 2000);
    chassis.moveToPoint(-43, -55, 2000,{.maxSpeed = 80});
    chassis.moveToPoint(-48, -49, 2000, {.forwards = false, .minSpeed = 20});
    chassis.turnToPoint(-55, -55, 2000, {.forwards = false});
    chassis.moveToPoint(-55, -55, 2000, {.forwards = false});
    pros::delay(600);
    clamp.set_value(true);
    

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
        
        case 7: 
            auton7();
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
            if (arm_state == 0 || arm_state == 2) {
                arm_state = 1;
            }
            else {
                arm_state = 2;
            }
            enable_pid = true;
            button_pressed = true;
        }
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
        if (!button_pressed) {
            arm_state = 0;
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
    isAuton = false;
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