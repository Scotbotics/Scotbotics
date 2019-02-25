//package that allows importing into the phone
package org.firstinspires.ftc.teamcode;

//import a lot of different functions, methods and commands for the ftc hardware

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpRoverRuckus", group="TeleOp")
public class TeleOpRoverRuckus extends OpMode
{
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 0.66;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    int encoderArm = 300;

    public DcMotor aDrive; // front left
    public DcMotor bDrive; // back left
    public DcMotor cDrive; // front right
    public DcMotor dDrive; // back right

    public DcMotor Lift;
    public DcMotor Arm;
    public DcMotor ArmExtender;
    public DcMotor Spindle;

    public Servo Servo;

    private ElapsedTime runtime = new ElapsedTime();        //Tells us when the 30 seconds is up

    public void init()
    {
        aDrive = hardwareMap.get(DcMotor.class, "aDrive");
        bDrive = hardwareMap.get(DcMotor.class, "bDrive");
        cDrive = hardwareMap.get(DcMotor.class, "cDrive");
        dDrive = hardwareMap.get(DcMotor.class, "dDrive");
        cDrive.setDirection(DcMotor.Direction.REVERSE);
        dDrive.setDirection(DcMotor.Direction.REVERSE);

        Lift = hardwareMap.get(DcMotor.class, "Lift");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        ArmExtender = hardwareMap.get(DcMotor.class, "ArmExtender");
        Spindle = hardwareMap.get(DcMotor.class, "Spindle");
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Servo = hardwareMap.get(Servo.class, "Servo");
    }
    @Override
    public void loop()
    {
        // DRIVING CONTROLS

        // Gives the stick special values for the mechamon wheel drive
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;

        //  Assigns a variable the motor power based off of special calculations
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        //  Gives the motors power so they turn
        aDrive.setPower(v1);
        cDrive.setPower(v2);
        bDrive.setPower(v3);
        dDrive.setPower(v4);

        // Lift
        if(gamepad2.dpad_up)
        {
            Lift.setPower(-1);
        }
        if(gamepad2.dpad_down)
        {
            Lift.setPower(1);
        }
        if(!gamepad2.dpad_up && !gamepad2.dpad_down)
        {
            Lift.setPower(0);
        }

        // Arm

        if(gamepad2.right_bumper){
            //leftFlipMotor.setTargetPosition(-300);
            Arm.setTargetPosition(encoderArm);

            // Turn On RUN_TO_POSITION
            // leftFlipMotor.setMode(DcMotor.RunMode.RUN_TO_POSIT ION);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //leftFlipMotor.setPower(0.25);
            Arm.setPower(0.20);
        }
        if(gamepad2.x) {
            Arm.setTargetPosition(encoderArm+300);

            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setPower(0.20);
        }

        if(gamepad2.a ) {
            Arm.setTargetPosition(encoderArm+750);

            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setPower(0.20);
        }

        ArmExtender.setPower(gamepad2.right_stick_y);

        // Spindle

        Spindle.setPower(gamepad2.left_stick_y);

        // Servo
        if(gamepad1.right_bumper)
        {
            Servo.setPosition(1); // turned 180 degrees
        }
        if(gamepad1.left_bumper)
        {
            Servo.setPosition(0); // turning back 180 degrees
        }
        if(!gamepad1.left_bumper && !gamepad1.right_bumper)
        {
            Servo.setPosition(0.5);
        }
    }

    public void stop()
    {
        aDrive.setPower(0);
        bDrive.setPower(0);
        cDrive.setPower(0);
        dDrive.setPower(0);
        Lift.setPower(0);
        Arm.setPower(0);
        ArmExtender.setPower(0);
        Spindle.setPower(0);
        Servo.setPosition(0.5);
    }

}
