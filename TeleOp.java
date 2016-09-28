package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class TeleOp extends OpMode {
	
	/*
	 * Note: the configuration of the servos is such that
	 * as the arm servo approaches 0, the arm position moves up (away from the floor).
	 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
	 */
    // TETRIX VALUES.

    // For the motor control - current joystick values, ideally encoder driven opening up assembly

    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor joint1Motor; // initial swing arm out, stays out
    DcMotor joint2Motor;
    DcMotor joint3Motor;
    DcMotor harvester;
    Servo box;



    // FOR NEW INVERSE KINEMATICS PROGRAMMING
    double Pr;
    double Aa;
    double Ab;
    double Ai;
    double A1;
    double A2;
    double L2;
    double L1;

    // encoder targets for teleop (1440 ppr for old tetrix motors, 280 for newer andymark ones)


    /**
     * Constructor
     */
    public TeleOp() {
        telemetry.addData("Team: " , "LETS DO THIS! GO TEAM GO!");
    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {

		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

        motorRight = hardwareMap.dcMotor.get("rightDrive");
        motorLeft = hardwareMap.dcMotor.get("leftDrive");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        joint1Motor = hardwareMap.dcMotor.get("joint1");

        joint2Motor = hardwareMap.dcMotor.get("joint2");
        joint2Motor.setDirection(DcMotor.Direction.REVERSE);

        harvester = hardwareMap.dcMotor.get("harvester");
        joint3Motor = hardwareMap.dcMotor.get("joint3");

        box = hardwareMap.servo.get("box"); // servo on the end of the arm

    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right
        // float throttle = -gamepad1.left_stick_y; // for non tank drive, with one stick controlling direcio and the other thtol
        // float direction = -gamepad1.left_stick_x;

        float right = gamepad1.right_stick_y;
        float left = gamepad1.left_stick_y;
        float joint2 = gamepad2.left_stick_y; //left stick
        float joint3 = gamepad2.right_stick_y;


        //float right = throttle - direction;
        //float left = throttle + direction;

        // clip the right/left values so that the values never exceed +/- 1

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        float rightPower = (float)scaleInput(right);
        float leftPower =  (float)scaleInput(left);

        float joint2Power = (float)scaleInput(joint2);
        float joint3Power = (float)scaleInput(joint3);


        motorRight.setPower(rightPower);
        motorLeft.setPower(leftPower);

        joint2Motor.setPower(joint2Power);
        joint3Motor.setPower(joint3Power);

        //motorLeft.setPower(left);
        //motorRight.setPower(right);


        if(gamepad2.a) {
            joint1Motor.setPower(-1.0);
        } else if(gamepad2.y) {
            joint1Motor.setPower(1.0);
        } else {
            joint1Motor.setPower(0.0);
        }

        if(gamepad1.right_bumper) {
            harvester.setPower(.3);
        } else if(gamepad1.left_bumper){
            harvester.setPower(-.3);
        } else {
            harvester.setPower(0.0);
        }

        if(gamepad2.x) {
            box.setPosition(1.0);
        } else if(gamepad2.b) {
            box.setPosition(0.0);
        } else {
            box.setPosition(.5);
        }


        // open up arm to normal position (starting at base) - maybe do during autonomous??
        // if(gamepad2.a) {
        //int joint1Pos = 0;
        //int joint2Pos = 0;
        //int joint3Pos = 0;

        //use encoder position to open up to normal position
        //joint1Motor.setTargetPosition(joint1Pos);
        //joint2Motor.setTargetPosition(joint2Pos);
        //joint3Motor.setTargetPosition(joint3Pos);

        //}


		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("j1", gamepad1.toString());
        telemetry.addData("j2",  gamepad2.toString());
    }


    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {
        motorLeft.setPower(0);
        motorRight.setPower(0);
        joint1Motor.setPower(0);
        joint2Motor.setPower(0);
        joint3Motor.setPower(0);
    }

    	
	/*
	 * This method scales the joystick input so for low joystick values, the 
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */

    //double scaleCubed (double val) {
    //	return Math.pow(val, 3);
    //}

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

    //KEEP IN MIND ALL ANGLES ARE IN RADIANS
    void calcAngles(float Px, float Py, DcMotor motor1, DcMotor motor2) {

        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");

        Pr = Math.sqrt(Math.pow(Px, 2) + Math.pow(Py, 2));
        Aa = Math.acos((Math.pow(L2, 2) - Math.pow(L1, 2) - Pr) / (-2.0 * L1 * Pr));
        Ab = Math.asin((Pr * Math.sin(Aa)) / L2);
        Ai = Math.asin(Py / Pr);

        A1 = Ai + Aa; // ANGLE OF FIRST JOINT
        A2 = Ab - Math.PI; //ANGLE OF SECOND JOINT

        //translate the angle of the joints into the encoder values


        double encoderTarget1 = (A1 / (2 * Math.PI)) * 280.0 * 6.0;
        double encoderTarget2 = (A2 / (2 * Math.PI)) * 1440.0 * 3.0;

    }
}
