
package org.firstinspires.ftc.robotcontroller.external.samples;

import android.app.Activity;
import android.graphics.Color;
import android.net.NetworkInfo;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 */
@Autonomous(name="FWDbot: Auto Drive By Encoder", group="FWDbot")

public class Launcherbot_FSM extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareLauncherBot2 robot   = new HardwareLauncherBot2();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.33 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.7;
    static final int        PRESS_RIGHT_BUTTON      = 3;
    static final int        PRESS_LEFT_BUTTON       = 2;
    static final int        HIT_BUTTON              = 1;
    static final int        LINE_FOLLOW             = 0;
    static final int        GO_OTHER_BEACON         = 4;
    static final int        ATTEMPT_SHOT            = 5;
    static final int        GO_IDLE                 = 6;
    static final int        LINE_FOLLOW2             = 7;
    static final int        HIT_BUTTON2              = 8;


    int state = LINE_FOLLOW;
    double distance_reading;
    boolean near_wall = false;
    View relativeLayout;

    ColorSensor colorSensor;    // Hardware Device Object
    double left_reading;
    double right_reading;
    double center_reading;
    double threshhold = 2.5;
    static final int UNKNOWN = 0;
    int left_side_color = UNKNOWN;
    int right_side_color = UNKNOWN;
    double hold = 0.22;
    int red_reading = 0;
    int blue_reading = 0;
    float hsvValues[] = {0F, 0F, 0F};

    boolean bPrevState = false;
    boolean bCurrState = false;
    boolean bLedOn = false;

    static final int Red_Team = 1;
    static final int Blue_Team = 2;
    static int Our_Team = Red_Team;
    static final double color_limit = 1.2;

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        colorSensor = hardwareMap.colorSensor.get("front_sensor_color");


        idle();


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.backleftMotor.getCurrentPosition(),
                robot.backrightMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            // What are we doing right now?
            switch (state) {
                case LINE_FOLLOW:
                    line_follow();
                    break;
                case HIT_BUTTON:
                    hit_button();
                    break;
                case ATTEMPT_SHOT:
                    attempt_shot();
                    break;
                case GO_IDLE:
                    go_idle();
                    break;
                case GO_OTHER_BEACON:
                    go_to_other_beacon();
                    break;
                case LINE_FOLLOW2:
                    line_follow();
                    break;
                case HIT_BUTTON2:
                    hit_button();
                    break;


            }
        }

    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.backleftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.backrightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.backleftMotor.setTargetPosition(newLeftTarget);
            robot.backrightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.backleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.backleftMotor.setPower(Math.abs(speed));
            robot.backrightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.backleftMotor.isBusy() && robot.backrightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.backleftMotor.getCurrentPosition(),
                        robot.backrightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.backleftMotor.setPower(0);
            robot.backrightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.backleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void line_follow() {
        bCurrState = gamepad1.x;
        if ((bCurrState == true) && (bCurrState != bPrevState)) {
            bLedOn = !bLedOn;
            colorSensor.enableLed(bLedOn);
        }
        bPrevState = bCurrState;
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        // change the background color to match the color detected by the RGB sensor
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method
        relativeLayout.post(new Runnable() {
            public void run() {relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));}
        });

        left_reading = robot.odsSensorLeft.getRawLightDetected();
        right_reading = robot.odsSensorRight.getRawLightDetected();
        center_reading = robot.odsSensorCenter.getRawLightDetected();


        if (left_reading < threshhold) {
            turn_left();
        } else if (right_reading < threshhold) {
            turn_right();
        } else if (center_reading < threshhold) {
            drive_backward();
        } else {
            drive_backward();
        }
        if (near_wall()){
            state = HIT_BUTTON;
        }

        // send the info back to driver station using telemetry function.
        telemetry.addLine("Line Following");
        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Left Reading", left_reading);
        telemetry.addData("Right Reading", right_reading);
        telemetry.addData("Center Reading", center_reading);
        telemetry.update();
    }

    public void hit_button() {
        // check the status of the x button on either gamepad.
        bCurrState = gamepad1.x;

        // check for button state transitions.
        if ((bCurrState == true) && (bCurrState != bPrevState)) {

            // button is transitioning to a pressed state. So Toggle LED
            bLedOn = !bLedOn;
            colorSensor.enableLed(bLedOn);
        }

        // update previous state variable.
        bPrevState = bCurrState;

        // convert the RGB values to HSV values.
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addLine("Button Hitting");
        telemetry.addData("LED", bLedOn ? "On" : "Off");
        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());

        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });

        telemetry.update();

        drive_forward();

        sleep(500);

        drive_backward();

        sleep(2000);

        if(colorSensor.blue() > colorSensor.red()) {
            drive_backward();
        } else {
            state = ATTEMPT_SHOT;
        }

    }



    public void go_to_other_beacon() {

        turn_left();
        sleep(2000);
        stop_drive();
        state = LINE_FOLLOW2;
    }
    public void attempt_shot() {
        drive_forward();
        sleep(1400);
        stop_drive();
        robot.pitcherMotorright.setPower(0.9);
        robot.pitcherMotorleft.setPower(0.9);
        robot.WaterMillMotor.setPower(-0.5);
        robot.RubberBandMotor.setPower(0.5);
        sleep(2000);
        robot.WaterMillMotor.setPower(0);
        robot.RubberBandMotor.setPower(0);
        robot.pitcherMotorright.setPower(0.0);
        robot.pitcherMotorleft.setPower(0.0);
        telemetry.addLine("Attempting shot");
        telemetry.addData("Path", "Complete");
        telemetry.update();
        drive_forward();
        sleep(1500);
        turn_left();
        sleep(700);
        drive_forward();
        sleep(1500);
        stop_drive();
        state = GO_IDLE;
    }

    public void line_follow2() {
        bCurrState = gamepad1.x;
        if ((bCurrState == true) && (bCurrState != bPrevState)) {
            bLedOn = !bLedOn;
            colorSensor.enableLed(bLedOn);
        }
        bPrevState = bCurrState;
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        // change the background color to match the color detected by the RGB sensor
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method
        relativeLayout.post(new Runnable() {
            public void run() {relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));}
        });

        left_reading = robot.odsSensorLeft.getRawLightDetected();
        right_reading = robot.odsSensorRight.getRawLightDetected();
        center_reading = robot.odsSensorCenter.getRawLightDetected();


        if (left_reading < threshhold) {
            turn_left();
        } else if (right_reading < threshhold) {
            turn_right();
        } else if (center_reading < threshhold) {
            drive_backward();
        } else {
            drive_backward();
        }
        if (near_wall()){
            state = HIT_BUTTON2;
        }

        // send the info back to driver station using telemetry function.
        telemetry.addLine("Line Following");
        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Left Reading", left_reading);
        telemetry.addData("Right Reading", right_reading);
        telemetry.addData("Center Reading", center_reading);
        telemetry.update();
    }

    public void hit_button2() {
        // check the status of the x button on either gamepad.
        bCurrState = gamepad1.x;

        // check for button state transitions.
        if ((bCurrState == true) && (bCurrState != bPrevState)) {

            // button is transitioning to a pressed state. So Toggle LED
            bLedOn = !bLedOn;
            colorSensor.enableLed(bLedOn);
        }

        // update previous state variable.
        bPrevState = bCurrState;

        // convert the RGB values to HSV values.
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addLine("Button Hitting");
        telemetry.addData("LED", bLedOn ? "On" : "Off");
        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());

        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });

        telemetry.update();

        drive_forward();

        sleep(500);

        drive_backward();

        sleep(2000);

        if(colorSensor.red() > colorSensor.blue()) {
            drive_backward();
        } else {
            state = GO_IDLE;
        }

    }




    void drive_forward() {
        robot.backleftMotor.setPower(hold);
        robot.backrightMotor.setPower(hold);
    }

    void drive_backward() {
        robot.backleftMotor.setPower(-hold);
        robot.backrightMotor.setPower(-hold);
    }

    void turn_left() {
        robot.backleftMotor.setPower(-hold);
        robot.backrightMotor.setPower(hold);
    }

    void turn_right() {
        robot.backleftMotor.setPower(hold);
        robot.backrightMotor.setPower(-hold);
    }

    void stop_drive() {
        robot.backleftMotor.setPower(0.0);
        robot.backrightMotor.setPower(0.0);
    }
    boolean near_wall(){
        distance_reading = robot.front_distance_sensor.getRawLightDetected();
        telemetry.addData("Front Reading", distance_reading);
        if (distance_reading > 0.12) {
            near_wall = true;
        } else {
            near_wall = false;
        }
        return near_wall;
    }
    boolean color_left(){
        // Amc stubs\
        return false;
    }
    boolean color_right(){
        // Amc stubs\
        return false;
    }
    public void go_idle(){
        drive_forward();
        sleep(200);
        stop_drive();
        idle();
        sleep(900000000);
    }

}
