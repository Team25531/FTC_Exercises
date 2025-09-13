package org.firstinspires.ftc.teamcode.sandbox.Aaron;


import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.sandbox.Aaron.BackgroundElbowMove;
import org.firstinspires.ftc.teamcode.sandbox.Aaron.GlobalState;

@Autonomous(name = "AutoScoreTestSideways", group = "auto")
//we need to add the DcMotors
public class AutoScoreTestSideways extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor;// = hardwareMap.dcMotor.get("frontLeft");
    private DcMotor backLeftMotor;// = hardwareMap.dcMotor.get("backLeft");
    private DcMotor frontRightMotor;// = hardwareMap.dcMotor.get("frontRight");
    private DcMotor backRightMotor;// = hardwareMap.dcMotor.get("backRight");
    private DcMotor extension;
    public static DcMotor elbow;// = hardwareMap.dcMotor.get("backRight");
    private CRServo finger;
    private static int elbowPosition = -200;

    double position = 0.312;
    GlobalState globalState = new GlobalState();
    private int tickTarget = 100;

    private double targetHeading = 0;
    private double driveSpeed = 0.3;
    private double turnSpeed = 0;
    private double headingError = 0;
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable.
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable.
    static final double HEADING_THRESHOLD = 1.0;    // How close must the heading get to the target before moving to next step.

    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private int leftTarget = 0;
    private int rightTarget = 0;
    int COUNTS_PER_INCH = 115;

    final double ARM_TICKS_PER_DEGREE = 28 // number of encoder ticks per rotation of the bare motor
            * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
            * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
            * 1 / 360.0; // we want ticks per degree, not per rotation


    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    private IMU imu = null;

    private void turn(DcMotor fL, DcMotor bL, DcMotor fR, DcMotor bR, double time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path 11", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            fL.setPower(0.5);
            fR.setPower(0.5);
            bL.setPower(0.5);
            bR.setPower(0.5);

        }
        runtime.reset();

    }

    private void straightLine(DcMotor fL, DcMotor bL, DcMotor fR, DcMotor bR, double time) {
        runtime.reset();
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path 11", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            fL.setPower(0.5);
            fR.setPower(0.5);
            bL.setPower(0.5);
            bR.setPower(0.5);

        }
        runtime.reset();
        StopMovement();

    }


    @Override
    public void runOpMode() throws InterruptedException {

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        extension = hardwareMap.dcMotor.get("extension");

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        extension.setDirection(DcMotor.Direction.REVERSE);

//      commenting out original direction.
//        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
//        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotor.Direction.FORWARD);


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elbow = hardwareMap.dcMotor.get("elbow");
        finger = hardwareMap.crservo.get("finger");


        // elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setDirection(DcMotor.Direction.REVERSE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        while (opModeInInit()) {
            sendTelemetry(true);
        }

        //waiting for start
        waitForStart();

        double driveSpeed = 0.5;
        driveSpeed = 0.3;

        //always starts from zero. Left is positive, Right is negative.
        MoveStraightTicks(200, driveSpeed);
        sleep(2000);
        MoveStraightRightTicks(1000,driveSpeed);
       // sleep(2000);
       // MoveStraightLeftTicks(2000,driveSpeed);
        //        int elbowTarget = -300;
//        elbow.setTargetPosition(elbowTarget);
//        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        elbow.setPower(-0.3);
//
//        sleep(10);
//
//        int heading = 135;
//        turnToHeading(driveSpeed, heading);
//        holdHeading(driveSpeed, heading, 1);
//
//        //this sets the current forward direction of the robot to zero, so the next turn command starts from zero.
//        resetHeadingIMU();
//        sleep(10);
//
//        MoveStraightTicks(670, driveSpeed);
//
//
//        elbowTarget = -2720;
//        elbow.setTargetPosition(elbowTarget);
//        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        elbow.setPower(-0.3);
//
//        while (elbow.getCurrentPosition() != elbowTarget) {
//            sleep(10);
//        }
//        sleep(100);
//        runtime.reset();
//        while (runtime.seconds() < 3) {
//            extension.setPower(0.5);
//        }
//        runtime.reset();
//        MoveStraightTicks(400, driveSpeed);
//        while (runtime.seconds() < 3) {
//            finger.setPower(0.9);
//        }
//        finger.setPower(0);
//        runtime.reset();
//        while (runtime.seconds() < 2) {
//            extension.setPower(-0.5);
//        }
//
//
//        //sometimes a little sleep time helps things from being to jittery.
//        sleep(10);
//
//        heading = -135;
//        turnToHeading(driveSpeed, heading);
//        holdHeading(driveSpeed, heading, 1);
//
//        sleep(10);
//
//        //NOTE: Reverse might not work properly right now, needs more testing.
//        //BE PREPARED TO STOP THE ROBOT WHEN TRYING TO REVERSE, IT MIGHT ACCELERATE QUICKLY AND FAIL TO STOP BY ITSELF
//        MoveStraightTicks(1742, driveSpeed);
//        sleep(10);
//
//        heading = -90;
//        turnToHeading(driveSpeed, heading);
//        holdHeading(driveSpeed, heading, 1);
//
//        sleep(10);
//        elbowTarget = -3380  ;
//        elbow.setTargetPosition(elbowTarget);
//        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        elbow.setPower(-0.3);
//
//
//        MoveStraightTicks(1567, driveSpeed);
//        sleep(10);

        requestOpModeStop();

//TODO: need start roller and stop roller methods, also release arm power (maybe just stop the routine).


        //Start and move arm to set down position
        //Forward 1; right ticks = 909, left tick = 848
        //Turn 1; heading = 125.4, right ticks = 1462, left ticks = -1236
        //Forward 2; right ticks = 735, left ticks = 538
        //Stop 1; 0,0,0
        //Arm raise; unfound
        //eject; finger 0.8 power
        //arm high set pos; unfound
        //Turn 2; heading = -151.8, right ticks = -1357, left ticks = 1521
        // Forward 3;right ticks = 2056, left ticks = 2115
        //Turn 3; Heading = 67.8, Right ticks -529, left ticks = 705;
        //Forward 4; right ticks = 247 , left ticks, 260


    }


    void SetElbowGoal(int goal) {
        elbowPosition = goal;
    }

    public static int GetElbowGoal() {
        return elbowPosition;
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    private void StopElbowControl() {
        globalState.isCancelled = true;
    }

    private void StartElbowControl() {
        System.out.println("StartElbowControl");
        new Thread(new BackgroundElbowMove(globalState)).start();
    }

    private void SetElbowPosition(int position) {
        globalState.setElbowPosition(position);

    }

    @SuppressLint("DefaultLocale")
    String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    @SuppressLint("DefaultLocale")
    String formatDegrees(double degrees) {
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    private double headingStore;

    public void turnToHeading(double maxTurnSpeed, double heading) {
        System.out.println("Methods:turnToHeading");
        headingStore = heading;
        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            System.out.println("turnSpeed:" + Double.toString(turnSpeed));

            if (Math.abs(turnSpeed) < .1) {
                if (turnSpeed > 0) {
                    turnSpeed = .1;
                } else {
                    turnSpeed = -.1;
                }
            }
            System.out.println("OutturnSpeed:" + Double.toString(turnSpeed));

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(true);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    private void StopMovement() {
        moveRobot(0, 0);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveRobot(double drive, double turn) {

        double leftSpeed = drive - turn;
        double rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        frontLeftMotor.setPower(leftSpeed);
        frontRightMotor.setPower(rightSpeed);
        backLeftMotor.setPower(leftSpeed);
        backRightMotor.setPower(rightSpeed);
        sendTelemetry(true);
    }

    public void moveRobotLeft(double drive, double turn) {
        double frontSpeed = drive - turn;
        double backSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(frontSpeed), Math.abs(backSpeed));
        if (max > 1.0) {
            frontSpeed /= max;
            backSpeed /= max;
        }

        frontLeftMotor.setPower(frontSpeed);
        frontRightMotor.setPower(backSpeed);
        backLeftMotor.setPower(backSpeed);
        backRightMotor.setPower(frontSpeed);
        sendTelemetry(true);
    }

    public void moveRobotRight(double drive, double turn) {
        double frontSpeed = drive - turn;
        double backSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(frontSpeed), Math.abs(backSpeed));
        if (max > 1.0) {
            frontSpeed /= max;
            backSpeed /= max;
        }

        frontLeftMotor.setPower(frontSpeed);
        frontRightMotor.setPower(backSpeed);
        backLeftMotor.setPower(backSpeed);
        backRightMotor.setPower(frontSpeed);
        sendTelemetry(true);
    }

    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            if (Math.abs(turnSpeed) < .08) {
                if (turnSpeed > 0) {
                    turnSpeed = .08;
                } else {
                    turnSpeed = -.08;
                }
            }
            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }
        // moveRobot(0, 0);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {

        // Determine the heading current error
        headingError = desiredHeading - getHeadingIMU();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        double correction = headingError * proportionalGain;
        return Range.clip(correction, -1, 1);
    }

    public double getSteeringCorrectionRight() {

        // Determine the heading current error
        double desiredHeading = 90;
        int offsetHeading = 90;
        double headingIMU = getHeadingIMU();
        telemetry.addData("headingIMU", "%4.3f", headingIMU);
        headingError = desiredHeading - (headingIMU + offsetHeading);
        telemetry.addData("heading Error before", "%4.3f", headingError);

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        telemetry.addData("heading Error after", "%4.3f", headingError);

        double proportionalGain = P_DRIVE_GAIN;
        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        double correction = headingError * proportionalGain;
        telemetry.addData("correction no clip", "%4.3f", correction);

        return Range.clip(correction, -1, 1);
    }

    public double getSteeringCorrectionLeft() {

        // Determine the heading current error
        double desiredHeading = -90;
        int offsetHeading = -90;
        double headingIMU = getHeadingIMU();
        telemetry.addData("headingIMU", "%4.3f", headingIMU);
        headingError = desiredHeading - (headingIMU + offsetHeading);
        telemetry.addData("heading Error before", "%4.3f", headingError);

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        telemetry.addData("heading Error after", "%4.3f", headingError);

        double proportionalGain = P_DRIVE_GAIN;
        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        double correction = headingError * proportionalGain;
        telemetry.addData("correction no clip", "%4.3f", correction);

        return Range.clip(correction, -1, 1);
    }

    private void MoveStraightRightTicks(int targetTicks, double speed) {
        if (opModeIsActive()) {
            resetHeadingIMU();
            tickTarget = targetTicks;
            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeftMotor.setTargetPosition(tickTarget);
            backLeftMotor.setTargetPosition(-tickTarget);
            frontRightMotor.setTargetPosition(-tickTarget);
            backRightMotor.setTargetPosition(tickTarget);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            moveRobotToTickTargetRight(speed);

        }
    }

    private void MoveStraightLeftTicks(int targetTicks, double speed) {
        if (opModeIsActive()) {
            resetHeadingIMU();
            tickTarget = targetTicks;
            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeftMotor.setTargetPosition(-tickTarget);
            backLeftMotor.setTargetPosition(tickTarget);
            frontRightMotor.setTargetPosition(tickTarget);
            backRightMotor.setTargetPosition(-tickTarget);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            moveRobotToTickTargetLeft(speed);

        }
    }

    private void MoveStraightTicks(int targetTicks, double speed) {
        if (opModeIsActive()) {
            resetHeadingIMU();
            tickTarget = targetTicks;
            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeftMotor.setTargetPosition(tickTarget);
            backLeftMotor.setTargetPosition(tickTarget);
            frontRightMotor.setTargetPosition(tickTarget);
            backRightMotor.setTargetPosition(tickTarget);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (tickTarget > 0) {
                moveRobotToTickTargetForward(speed);
            } else {
                moveRobotToTickTargetReverse(speed);
            }

        }
    }

    public int getAverageTicksMoved() {

        int total = getFrontLeftMotorTicks() +
                getFrontRightMotorTicks() +
                getBackLeftMotorTicks() +
                getBackRightMotorTicks();
        total = total / 4;
        return total;
    }


    public void moveRobotToTickTargetReverse(double speed) {
        while (opModeIsActive() && getAverageTicksMoved() > tickTarget) {

            double turnSpeed = getSteeringCorrection(0, P_DRIVE_GAIN);
            turnSpeed *= -1.0;

            moveRobot(-speed, turnSpeed);
        }
        StopMovement();
    }


    public void moveRobotToTickTargetForward(double speed) {
        while (opModeIsActive() && getAverageTicksMoved() < tickTarget) {
            double turnSpeed = getSteeringCorrection(0, P_DRIVE_GAIN);

            // if driving in reverse, the motor correction also needs to be reversed
            //   if (tickTarget < 0) turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            moveRobot(speed, turnSpeed);
            //  moveRobot(.6, 0);
        }
        // moveRobot(0, 0);
        StopMovement();
    }

    public void moveRobotToTickTargetRight(double speed) {
        while (opModeIsActive() && getAverageTicksMoved() < tickTarget) {
            turnSpeed = getSteeringCorrectionRight();
            //  telemetry.addData("turnSpeed", turnSpeed);
            // if driving in reverse, the motor correction also needs to be reversed
            //   if (tickTarget < 0) turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            moveRobotRight(speed, turnSpeed);
            //  moveRobot(.6, 0);
        }
        // moveRobot(0, 0);
        StopMovement();
    }
    public void moveRobotToTickTargetLeft(double speed) {
        while (opModeIsActive() && getAverageTicksMoved() < tickTarget) {
            turnSpeed = getSteeringCorrectionLeft();
            //  telemetry.addData("turnSpeed", turnSpeed);
            // if driving in reverse, the motor correction also needs to be reversed
            //   if (tickTarget < 0) turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            moveRobotLeft(speed, turnSpeed);
            //  moveRobot(.6, 0);
        }
        // moveRobot(0, 0);
        StopMovement();
    }

    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            //  telemetry.addData("Target Pos L:R", "%7d:%7d", leftTarget, rightTarget);
            telemetry.addData("Actual Pos L:R", "%7d:%7d", frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        int frontLeftMotorTicks = getFrontLeftMotorTicks();
        int backLeftMotorTicks = getBackLeftMotorTicks();
        int frontRightMotorTicks = getFrontRightMotorTicks();
        int backRightMotorTicks = getBackRightMotorTicks();
        int averageTicks = (frontLeftMotorTicks + backLeftMotorTicks + frontRightMotorTicks + backRightMotorTicks) / 4;
        telemetry.addData("> heading IMU", "%4.1f", getHeadingIMU())
                //  .addData(">heading NavX", "%4.1f", getHeadingNavX())
                .addData(">goal heading ", "%4.1f", headingStore)
                .addData("> delta heading", "%4.1f", headingError)
                .addData("> turn speed", "%4.1f", turnSpeed)
                .addData(">frontLeftMotor ticks", frontLeftMotorTicks)
                .addData(">backLeftMotor ticks", backLeftMotorTicks)
                .addData(">frontRightMotor ticks", frontRightMotorTicks)
                .addData(">backRightMotor ticks", backRightMotorTicks)
                .addData(">Average Motor ticks", averageTicks)
                .addData("> elbow Pos", elbow.getCurrentPosition())
                .addData("> Elbow State", globalState.getElbowPosition());
        telemetry.update();
    }

    private int getBackRightMotorTicks() {
        return backRightMotor.getCurrentPosition();
    }

    private int getFrontRightMotorTicks() {
        return frontRightMotor.getCurrentPosition();
    }

    private int getBackLeftMotorTicks() {
        return backLeftMotor.getCurrentPosition();
    }

    private int getFrontLeftMotorTicks() {
        return frontLeftMotor.getCurrentPosition();
    }

    public void resetHeadingIMU() {
        imu.resetYaw();
    }

    public double getHeadingIMU() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }


}