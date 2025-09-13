package org.firstinspires.ftc.teamcode.sandbox.neel;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Robot2 FTC comp", group = "Ftc Comp")
//we need to add the DcMotors
@Disabled
public class ftcCompRobot2 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor;// = hardwareMap.dcMotor.get("frontLeft");
    private DcMotor backLeftMotor;// = hardwareMap.dcMotor.get("backLeft");
    private DcMotor frontRightMotor;// = hardwareMap.dcMotor.get("frontRight");
    private DcMotor backRightMotor;// = hardwareMap.dcMotor.get("backRight");
    private DcMotor elbow;// = hardwareMap.dcMotor.get("backRight");
    private DcMotor extension;
    private CRServo finger;


    double position = 0.312;
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation


    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN   = 0.8333;
    final double WRIST_FOLDED_OUT  = 0.5;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;
    int HoldPosition;


    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");



        DcMotor elbow = hardwareMap.dcMotor.get("elbow");
        finger = hardwareMap.crservo.get("finger");

        //setting direction for motors


        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        position = 0;//remove
        double addposition = 0.001;
        double addedcurrentPosition = position;
        double subposition = -0.001;
        double subcurrentPosition = position;
        int armPosition = 0;
        double armMovePos = 0.001;
        boolean gravityCondition = true;

        elbow.setDirection(DcMotor.Direction.REVERSE);
        //make it so that motors don't fall automaticly
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        //waiting for start
        waitForStart();
        //  elbow.setTargetPosition(0);
        //  elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //  elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //reseting variable called runtime
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double speedModifier = Math.abs(1.5);
            denominator *= speedModifier;

            if (gamepad1.y) {
                denominator *= 2.5;
            }

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);







            runtime.reset();


            telemetry.addData("Currently goal", HoldPosition);
            telemetry.addData("Currently   at", elbow.getCurrentPosition());
            telemetry.addData("Gravity code is now ", gravityCondition);

            if (gamepad1.right_bumper) {
                elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elbow.setPower(-0.9);
                HoldPosition = elbow.getCurrentPosition();
            } else if (gamepad1.right_trigger > 0) {
                elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elbow.setPower(0.9);
                telemetry.addData("currently at :", elbow.getCurrentPosition());
                HoldPosition = elbow.getCurrentPosition();
            } else {
                if (!endGame) {
                    elbow.setPower(0);
                    if (gravityCondition) {
                        HoldArmStill(HoldPosition, elbow);
                    }
                }
            }

            telemetry.update();


            //code for moving the finger aka the rubber tire with while loops
            if (gamepad1.left_bumper) {

                finger.setPower(0.8);
                //tire moves inward to pull block in
            } else if (gamepad1.left_trigger > 0) {
                //tire moves outward to push block out
                finger.setPower(-0.8);
            } else {
                finger.setPower(0);
            }

            if (gamepad1.a) {
                if (gravityCondition) {
                    gravityCondition = false;
                }

            }

            if (gamepad1.dpad_down) {
                endGame =true;

                elbow.setPower(0.9);

            }



        }
    }
    boolean endGame = false;

    public void HoldArmStill(int posToHold, DcMotor motor) {
        int tolerance = 4;
        int currentPosition = motor.getCurrentPosition();
        if ((Math.abs(currentPosition - posToHold)) < tolerance) {
            return;
        }
        motor.setTargetPosition(posToHold);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(Math.abs(0.8));
    }
//    public void StopArmMoving ( DcMotor motor){
//        motor.setTargetPosition(800);
//        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//    }

}