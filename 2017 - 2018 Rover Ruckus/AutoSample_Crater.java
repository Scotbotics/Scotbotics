
package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="AutoSampleCrater", group="Autononmous")
public class AutoSample_Crater extends LinearOpMode {

    /* Declare OpMode members. */
    
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 0.66;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // ALL THE MOTORS ON THE DRIVE SYSTEM
    private DcMotor aDrive = null;              //Front Left
    private DcMotor bDrive = null;              //Back Left
    private DcMotor cDrive = null;              //Front right
    private DcMotor dDrive = null;              //Back right

    //FUNCTIONS MOTORS
    public DcMotor Arm;                         //Moves the arm up and down
    public DcMotor ArmExtender;                 //Makes the arm grow in length
    public DcMotor Spindle;                     //Spins the collection spindle
    public DcMotor Lift;                        //Gets the robot on and off the lander

    //SERVOS
    public Servo Cover;                         //Covers the box that the minerals are in

    //VUFORIA STUFF
    private GoldAlignDetector detector;         //Makes the phone look for the gold
    double goldPos = 0;                         //The angle the gold is away from the robot

    //TIMERS
    private ElapsedTime runtime = new ElapsedTime();        //Tells us when the 30 seconds is up

    @Override
    public void runOpMode() {

        //PUTS THE NAMES OF THE MOTORS INTO THE CODE

        //DRIVE Motors
        aDrive = hardwareMap.get(DcMotor.class, "aDrive");
        bDrive = hardwareMap.get(DcMotor.class, "bDrive");
        cDrive = hardwareMap.get(DcMotor.class, "cDrive");
        dDrive = hardwareMap.get(DcMotor.class, "dDrive");
        cDrive.setDirection(DcMotor.Direction.REVERSE);
        dDrive.setDirection(DcMotor.Direction.REVERSE);

        aDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        aDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //FUNCTIONS Motors
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        ArmExtender = hardwareMap.get(DcMotor.class, "ArmExtender");
        Spindle = hardwareMap.get(DcMotor.class, "Spindle");
        Lift = hardwareMap.get(DcMotor.class, "Lift");

        //SERVOS
        Cover = hardwareMap.get(Servo.class, "Cover");

        //Vuforia
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Phone Stuff
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;
        detector.enable();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        //encoderLift(0.5, 8, 3);
        //encoderDrive(0.5, -2, -2, -2, -2 ,1);
        //Lift.setPower(0.5);
        //sleep(1800);
        //backward(0.25,200);
        //stop(1000);
        driftright(0.25, 600); // might take a bit longer or shorter?
        stop(1000);
        //forward(0.25, 200); // might take a bit longer or shorter?

        telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral

        goldPos = detector.getXPosition();  // Gets gold block posistion

        if(goldPos > 50 && goldPos < 600) {
            turnLeft(0.25, 650);
            stop(1000);
            backward(0.25, 700);
            sleep(100000000);
        }
        turnLeft(0.25,400);
        stop(1000);
        goldPos = detector.getXPosition();
        stop(1000);
        if(goldPos >  50 && goldPos < 600) {
            turnLeft(0.25, 100);
            stop(500);
            driftright(0.25, 700);
            sleep(100000000);
        }
        turnRight(0.25, 700);
        stop(1000);
        goldPos = detector.getXPosition();
        stop(1000);
        if(goldPos >  50 && goldPos < 550) {
            turnLeft(0.25, 100);
            stop(500);
            driftright(0.25, 700);
            sleep(100000000);
        }

    }

        //Mechanum Wheel Commands

    public void stop (long time)
    {
        telemetry.addData("status", "turnLeft");
        telemetry.update();

        aDrive.setPower(0);
        bDrive.setPower(0);
        cDrive.setPower(0);
        dDrive.setPower(0);
        sleep(time);

    }
    public void turnLeft (double speed, long time) {
            telemetry.addData("status", "turnLeft");
            telemetry.update();

            aDrive.setPower(-speed);
            bDrive.setPower(-speed);
            cDrive.setPower(speed);
            dDrive.setPower(speed);
            sleep(time);

        }

        public void turnRight (double speed, long time) {
            telemetry.addData("status", "turnRight");
            telemetry.update();

            aDrive.setPower(speed);
            bDrive.setPower(speed);
            cDrive.setPower(-speed);
            dDrive.setPower(-speed);
            sleep(time);

        }

        public void forward (double speed, long time) {
            telemetry.addData("status", "forward");
            telemetry.update();

            aDrive.setPower(speed);
            bDrive.setPower(speed);
            cDrive.setPower(speed);
            dDrive.setPower(speed);
            sleep(time);

        }

        public void backward (double speed, long time) {
            telemetry.addData("status", "forward");
            telemetry.update();

            aDrive.setPower(-speed);
            bDrive.setPower(-speed);
            cDrive.setPower(-speed);
            dDrive.setPower(-speed);
            sleep(time);

        }

        public void driftleft (double speed, long time) {
            telemetry.addData("status", "forward");
            telemetry.update();

            aDrive.setPower(-speed);
            bDrive.setPower(speed);
            cDrive.setPower(speed);
            dDrive.setPower(-speed);
            sleep(time);

        }

        public void driftright (double speed, long time) {
            telemetry.addData("status", "forward");
            telemetry.update();

            aDrive.setPower(speed);
            bDrive.setPower(-speed);
            cDrive.setPower(-speed);
            dDrive.setPower(speed);
            sleep(time);

        }

    public void encoderDrive(double speed,
                             double leftFront, double rightFront, double rightRear, double leftRear,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newRearLeftTarget;
        int newFrontRightTarget;
        int newRearRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            //parker = hardwareMap.dcMotor.get("frontLeft");
            //louis = hardwareMap.dcMotor.get("frontRight");
            //maria = hardwareMap.dcMotor.get("rearLeft");
            //matthew = hardwareMap.dcMotor.get("rearRight");
            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = aDrive.getCurrentPosition() + (int) (leftFront * COUNTS_PER_INCH);
            newRearLeftTarget = bDrive.getCurrentPosition() + (int) (leftRear * COUNTS_PER_INCH);
            newFrontRightTarget = cDrive.getCurrentPosition() + (int) (rightFront * COUNTS_PER_INCH);
            newRearRightTarget = dDrive.getCurrentPosition() + (int) (rightRear * COUNTS_PER_INCH);

            bDrive.setTargetPosition(newRearLeftTarget);
            aDrive.setTargetPosition(newFrontLeftTarget);
            dDrive.setTargetPosition(newRearRightTarget);
            cDrive.setTargetPosition(newFrontRightTarget);


            // Turn On RUN_TO_POSITION
            dDrive.setPower(rightRear / Math.abs(rightRear));
            bDrive.setPower(leftRear / Math.abs(leftRear));
            cDrive.setPower(rightFront / Math.abs(rightFront));
            aDrive.setPower(leftFront / Math.abs(leftFront));

            sleep(250);
            runtime.reset();

            /*
            aDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            cDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            dDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            */


            // reset the timeout time and start motion.

            // keep looping while we are still active, and there is time left, and all motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (cDrive.isBusy() && aDrive.isBusy() && bDrive.isBusy() && dDrive.isBusy())) {
                // Display it for the driver.
                //telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newRearLeftTarget, newRearRightTarget);
                //telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                //aDrive.getCurrentPosition(),
                //cDrive.getCurrentPosition(),
                //dDrive.getCurrentPosition(),
                //bDrive.getCurrentPosition());

                //telemetry.update();

                // Allow time for other processes to run.
                //sleep(1000);
            }


            // Turn off RUN_TO_POSITION
            /*
            if((bDrive.getCurrentPosition() == newRearLeftTarget) && (dDrive.getCurrentPosition() == newRearRightTarget)) {
                bDrive.setPower(0);
                aDrive.setPower(0);
                dDrive.setPower(0);
                cDrive.setPower(0);
            }
            */

            bDrive.setPower(0);
            aDrive.setPower(0);
            dDrive.setPower(0);
            cDrive.setPower(0);

            /*
            bDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            dDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            cDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            aDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            */

            sleep(250);   // optional pause after each move

        }
    }

        public void encoderLift(double speed, double move, double timeoutS) {
            int newMove;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newMove = Lift.getCurrentPosition() + (int) (move * COUNTS_PER_INCH);


                Lift.setTargetPosition(newMove);
                // Turn On RUN_TO_POSITION
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                Lift.setPower(Math.abs(speed));

                // keep looping while we are still active, and there is time left, and all motors are running.
                while (opModeIsActive() && (runtime.seconds() < timeoutS) && (Lift.isBusy())) {
                    // Display it for the driver.
                    telemetry.addData("Toby", "Actually Running",
                            Lift.getCurrentPosition());

                    telemetry.update();

                    // Allow time for other processes to run.
                    sleep(250);
                }
                Lift.setPower(0);

                // Turn off RUN_TO_POSITION
                Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                sleep(250);   // optional pause after each move
            }
    }
}

