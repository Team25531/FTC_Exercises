/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.neel;

import android.util.Size;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.List;

/*
 *  This OpMode illustrates the concept of driving an autonomous path based on Gyro (IMU) heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code uses the Universal IMU interface so it will work with either the BNO055, or BHI260 IMU.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *  The REV Logo should be facing UP, and the USB port should be facing forward.
 *  If this is not the configuration of your REV Control Hub, then the code should be modified to reflect the correct orientation.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: This code implements the requirement of calling setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver Station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  https://ftc-docs.firstinspires.org/field-coordinate-system
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name = "auto Test", group = "Trying")
//@Disabled
public class autoTest extends LinearOpMode {

    /* Declare OpMode members. */
//    private DcMotor         leftDrive   = null;
//    private DcMotor         rightDrive  = null;

    private DcMotorEx outtake;
    private CRServo intake;
    private CRServo storageWheel;
    private AprilTagProcessor aprilTag;
    public static final String WEBCAM_NAME = "Webcam 1";
    public static final int TARGET_TAG_ID = 24;
    public static int IDLE_VELOCITY = 600;

    private ElapsedTime storageTimer = new ElapsedTime();
    private VisionPortal visionPortal;
    int goalVelocity = 0;
    double range = 0.05;
    double distanceToTarget = 0;
    double angleToTarget = 0;
    double currentVelocity = 0;
    double currentPower = 0;
    private int frontLeftTarget = 0;
    private int backLeftTarget = 0;
    private int frontRightTarget = 0;
    private int backRightTarget = 0;
    boolean isIdleEnabled = false;

    //boolean useOuttake = true;
    boolean isShooting = false;
    boolean isAutoAimEnabled = true;
    boolean isAtGoalVelocity = false;
    boolean shooterNeedsReset = false;
    boolean isAimedAtTarget = false;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private IMU imu = null;      // Control/Expansion Hub IMU

    private double headingError = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private int leftTarget = 0;
    private int rightTarget = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 537.7; // 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static double DRIVE_SPEED = 0.6;     // Max driving speed for better distance accuracy.
    static final double TURN_SPEED = 0.3;     // Max turn speed to limit turn rate.
    static final double HEADING_THRESHOLD = 1.0;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable.
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable.

    //IntegratingGyroscope gyro;
    // NavxMicroNavigationSensor navxMicro;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {


        initializeMotors();
        initializeTagProcessor();

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        //todo: these 3 are needed for the navx init.
//        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
//        gyro = (IntegratingGyroscope)navxMicro;
//        navxMicro.initialize();

        // If you're only interested int the IntegratingGyroscope interface, the following will suffice.
        // gyro = hardwareMap.get(IntegratingGyroscope.class, "navx");

        // The gyro automatically starts calibrating. This takes a few seconds.
        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        //todo: this initializes the navx.
//        // Wait until the gyro calibration is complete
//        timer.reset();
//        while (navxMicro.isCalibrating())  {
//            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
//            telemetry.update();
//            Thread.sleep(50);
//        }
        imu.resetYaw();

        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        telemetry.update();

        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            // AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
            //  Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addData(">", "Imu int = %4.1f", getHeading());
//                    .addData(">", "Imu fa = %4.1f", angles.firstAngle)
//                    .addData("Imu ext = ", formatAngle(angles.angleUnit, angles.firstAngle));
            int ticks = frontLeftMotor.getCurrentPosition(); //todo: this only reads a single wheel.
            telemetry.addData("> ticks", ticks);
            telemetry.update();
        }

        // Set the encoders for closed loop speed control, and reset the heading.


        // Step through each leg of the path,
        // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
        //          holdHeading() is used after turns to let the heading stabilize
        //          Add a sleep(2000) after any step to keep the telemetry data visible for review
        //backing away from goal
        imu.resetYaw();
        shooterNeedsReset = false;


        DRIVE_SPEED = 0.3;
        if (isStopRequested()) return;

        driveStraight(DRIVE_SPEED, -50, 0.0);
        distanceToTarget = getDistanceToTag(24);
        if (distanceToTarget < 50) {
            int awayFromTarget = (int) (50 - distanceToTarget);
            driveStraight(DRIVE_SPEED, -awayFromTarget, 0);

        }
        setGoalVelocity();


        distanceToTarget = getDistanceToTag(24);
        shooterNeedsReset = false;

        if (distanceToTarget >= 50) {


//                driveStraight(DRIVE_SPEED, 0, 0.0);
            intake.setPower(-1);
            checkIfShooting();
            setGoalVelocity();
            runOuttakeMotor();

            resetRuntime();
            double runtime = getRuntime();
            while (runtime <= 6) {
                runOuttakeMotor();
                doShooting();
                runtime = getRuntime();
                telemetry.addData("Motor running", -0);
                telemetry.update();
            }

            storageWheel.setPower(0);
            outtake.setPower(0);
            isShooting = false;


            imu.resetYaw();

            turnToHeading(TURN_SPEED, -55);
            holdHeading(TURN_SPEED, -55, .5);

            // 4. go very slowly towards the balls
            DRIVE_SPEED = 0.1
            ;
           // intake.setPower(-1);
            imu.resetYaw();
            driveStraight(DRIVE_SPEED, 45, 0.0);
            imu.resetYaw();
            DRIVE_SPEED = 0.3;
            driveStraight(DRIVE_SPEED, -40, 0.0);
            imu.resetYaw();
            turnToHeading(TURN_SPEED, 55);
            holdHeading(TURN_SPEED, 55, .5);
            distanceToTarget = getDistanceToTag(24);
            resetRuntime();
            checkIfShooting();
            setGoalVelocity();
            resetRuntime();
            runtime = getRuntime();
            while (runtime <= 10) {
                runOuttakeMotor();
                doShooting();
                runtime = getRuntime();
                telemetry.addData("Motor running", -0);
                telemetry.update();
            }


        }

        telemetry.update();


//        while (opModeIsActive()) {
//            double ve;
//
//            ve = (int) ((-0.0861 * Math.pow(distanceToTarget, 2)) + (20.729 * distanceToTarget) + 104.51);
//            telemetry.addData("current velocity", ve);
//
//            if (isStopRequested()) return;
//
//            // Assume start from near the depot
//            // 1. first go back 35 inches
//            // 2. shoot two balls already loaded
//            while (distanceToTarget <= 40) {
//                driveStraight(DRIVE_SPEED, -40, 0.0);
//                setGoalVelocity();
//                distanceToTarget = getDistanceToTag(24);
//                telemetry.addData("distanceToTarget", distanceToTarget);
//                telemetry.update();
//            }
//
//            distanceToTarget = getDistanceToTag(24);
//
//            if (distanceToTarget >=40)
//            {
//                driveStraight(DRIVE_SPEED, 0, 0.0);
//
//
//                intake.setPower(-1);
//                setGoalVelocity();
//                runOuttakeMotor();
//                checkIfShooting();
//
////                while (!isAimedAtTarget) {
////                    telemetry.addData("Angle To Target: ", angleToTarget);
////                    telemetry.update();
////                    aimToTarget();
////                }
//
//                doShooting();
//
//                if (shooterNeedsReset) {
//                    resetRuntime();
//                    double runtime = getRuntime();
//                    while (runtime < 4) {
//                        runtime = getRuntime();
//                    }
//            }
//
//            // Check distance again
////            distanceToTarget = getDistanceToTag(24);
////            shooterNeedsReset = false;
////
////            telemetry.addData("isAimed", isAimedAtTarget);
////            telemetry.update();
////            isShooting = true; // get ready for shooting
//
//
////             correct the angle
//
//
////            intake.setPower(-1);
////            setGoalVelocity();
////            runOuttakeMotor();
////            checkIfShooting();
////
////            doShooting();
//
//
//            }
//            // ensure we're not moving while shooting.
////            resetRuntime();
////            double runtime = getRuntime();
////            while (runtime < 4) {
////                runtime = getRuntime();
////            }
////            imu.resetYaw();
////
////            // 3. Turn towards first row of balls
////            turnToHeading(TURN_SPEED, -45);
////            holdHeading(TURN_SPEED, -45, .5);
////            imu.resetYaw();
////
////            // 4. go very slowly towards the balls
////            DRIVE_SPEED = 0.2;
////            intake.setPower(-1);
////            driveStraight(DRIVE_SPEED, 35, 0.0);
////            imu.resetYaw();
////            DRIVE_SPEED = 0.6;
////            driveStraight(DRIVE_SPEED, -35, 0.0);
////            imu.resetYaw();
////            turnToHeading(TURN_SPEED, 45);
////            holdHeading(TURN_SPEED, 45, .5);
////            imu.resetYaw();
////            isShooting = true;
////            while (!isAimedAtTarget) {
////                telemetry.addData("Angle To Target: ", angleToTarget);
////                telemetry.update();
////
////                aimToTarget();
////            }
////            intake.setPower(-1);
////            setGoalVelocity();
////            runOuttakeMotor();
////            checkIfShooting();
////
////            doShooting();
//
//
//        }
////turn towards ball
//
//
//        //drive to ball and pick up
//        //ADJUST DRIVE SPEED
//        //DRIVE_SPEED = 0.2
//        //INTAKE
//        //intake.setPower(-1)
//        //driveStraight(DRIVE_SPEED, 55, 0.0);
//
//
////        imu.resetYaw()
////        //go back to shooting line
////        DRIVE_SPEED = 0.6
////        driveStraight(DRIVE_SPEED, -55, 0.0);
//
////        imu.resetYaw();
//        //angle to goal then shoot
////        turnToHeading(TURN_SPEED, 45);
////        holdHeading(TURN_SPEED, 45, .5);
//        //angle
//        //angeToTarget();
//        //SHOOT NOW
//        //setGoalVelocity();
//        //runOuttakeMotor();
////        imu.resetYaw();
//

        telemetry.addData("Path", "Complete");
        telemetry.update();
        //sleep(5000);  // Pause to display last telemetry message.
    }


    private void initializeMotors() {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        intake = hardwareMap.crservo.get("intake");
        storageWheel = hardwareMap.crservo.get("storageWheel");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        //TODO: fix this to pull the correct object.
        //outtake = hardwareMap.get(DcMotorEx.class, "frontLeft");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
     * Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance      Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading       Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                      0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                      If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {
        ResetEncoders();
        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            frontLeftTarget = frontLeftMotor.getCurrentPosition() + moveCounts;
            backLeftTarget = backLeftMotor.getCurrentPosition() + moveCounts;
            frontRightTarget = frontRightMotor.getCurrentPosition() + moveCounts;
            backRightTarget = backRightMotor.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            frontLeftMotor.setTargetPosition(frontLeftTarget);
            backLeftMotor.setTargetPosition(backLeftTarget);
            frontRightMotor.setTargetPosition(frontRightTarget);
            backRightMotor.setTargetPosition(backRightTarget);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (frontLeftMotor.isBusy() && frontRightMotor.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }


    /**
     * Spin on the central axis to point in a new direction.
     * <p>
     * Move will stop if either of these conditions occur:
     * <p>
     * 1) Move gets to the heading (angle)
     * <p>
     * 2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     * Obtain & hold a heading for a finite amount of time
     * <p>
     * Move will stop once the requested time has elapsed
     * <p>
     * This function is useful for giving the robot a moment to stabilize its heading between movements.
     *
     * @param maxTurnSpeed Maximum differential turn speed (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     * @param holdTime     Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    private void ResetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading   The desired absolute heading (relative to last heading reset)
     * @param proportionalGain Gain factor applied to heading error to obtain turning power.
     * @return Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     *
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed = drive - turn;
        rightSpeed = drive + turn;

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
    }

    /**
     * Display the various control parameters while driving
     *
     * @param straight Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R", "%7d:%7d", leftTarget, rightTarget);
            telemetry.addData("Actual Pos L:R", "%7d:%7d", frontLeftMotor.getCurrentPosition(),
                    frontRightMotor.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr", "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    private void doShooting() {
        //if we are at goal, then run the storageWheel.
        if (isAtGoalVelocity) {
            storageTimer.reset();
            storageWheel.setPower(-1);
            //todo: determine correct duration for this timer.
            while (opModeIsActive() & storageTimer.milliseconds() < 4000 && !shooterNeedsReset) {
                sleep(1);
                //todo: delete this telemetry.
                runOuttakeMotor();
                telemetry.addData("velocity", goalVelocity);
                telemetry.addData("curPower", currentPower);
                telemetry.update();
            }
            shooterNeedsReset = true;
        }
        telemetry.addData("shooterNeedsReset", shooterNeedsReset);
    }

    private void checkIfShooting() {
        //to shoot, hold down the left_trigger.

        isShooting = true;


    }


    private void setGoalVelocity() {
        //only compute velocity if we're actually shooting.
        int tempVelocity = goalVelocity;
        if (distanceToTarget > 40 && distanceToTarget < 140) {
            telemetry.addData("in loop", 0);
            tempVelocity = (int) (693.198761 + 1191.999926 * (1.0 - Math.exp(-0.007992 * distanceToTarget))+25);
            telemetry.addData("tempVelocity", tempVelocity);
        }

        if (isShooting && !shooterNeedsReset) {
            goalVelocity = tempVelocity;
        }


        telemetry.addData("goalVelocity", goalVelocity);
    }


    //DATA POINTS //61 1018 //79 1200//65 1080//60 1040//68 1110// 70 1150// 125 1350
    //NEW DATA ///


    private void runOuttakeMotor() {
        currentVelocity = outtake.getVelocity();
        currentPower = outtake.getPower();
        telemetry.addData("curVelocity", currentVelocity);
        telemetry.addData("curPower", currentPower);

        double minRange = goalVelocity - (goalVelocity * range);
        double maxRange = goalVelocity + (goalVelocity * range);

        if (currentVelocity < minRange) {
            currentPower = currentPower + 0.001;
        }
        if (currentVelocity > maxRange) {
            currentPower = currentPower - 0.001;
        }
        isAtGoalVelocity = (currentVelocity < maxRange && currentVelocity > minRange);
        telemetry.addData("isAtGoalVelocity", isAtGoalVelocity);
        if (isAtGoalVelocity) {
            return;
        }

        outtake.setPower(currentPower);
    }


    private void doDriving() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        angleToTarget = getAngleToTag(TARGET_TAG_ID);
        distanceToTarget = getDistanceToTag(TARGET_TAG_ID);
        telemetry.addData("dist  goal", distanceToTarget);
        telemetry.addData("angle goal", angleToTarget);

        //if shooting and the camera is working then override driver input
        //and try to steer towards the target.
        if (isShooting && isAutoAimEnabled && !shooterNeedsReset) {
            isAimedAtTarget = !(angleToTarget < -1 || angleToTarget > 1);
            if (!isAimedAtTarget) {
                if (angleToTarget < -1) rx = 0.3;
                if (angleToTarget > 1) rx = -0.3;
            }
            telemetry.addData("isAimedAtTarget", isAimedAtTarget);
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        //todo: enable this.
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    private int getDistanceToTag(int tagID) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        int range = 0;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == tagID) {
                range = (int) detection.ftcPose.range;
                break;
            }
        }
        return range;
    }

    private void aimToTarget() {
        double y = 0;
        double x = 0;
        double rx = 0;

        telemetry.addData("isShooting: ", isShooting);
        telemetry.addData("isAutoAimEnabled: ", isAutoAimEnabled);
        telemetry.addData("shooterNeedsReset: ", shooterNeedsReset);
        telemetry.update();

        if (isShooting && isAutoAimEnabled && !shooterNeedsReset) {
            angleToTarget = getAngleToTag(24);
            isAimedAtTarget = (angleToTarget > -1 && angleToTarget < 1);
            if (!isAimedAtTarget) {
                if (angleToTarget < -1) rx = 0.3;
                if (angleToTarget > 1) rx = -0.3;
            }

            telemetry.addData("isAimedAtTarget", isAimedAtTarget);
        } else {
            telemetry.addData("isShooting: ", isShooting);
            telemetry.addData("isAutoAimEnabled: ", isAutoAimEnabled);
            telemetry.addData("shooterNeedsReset: ", shooterNeedsReset);
            telemetry.addData("WTF!!!!", 0);
            telemetry.update();
            while (true) {
            }

        }
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        //todo: enable this.
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

    }


    private int getAngleToTag(int tagID) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        int angle = 0;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == tagID) {
                angle = (int) detection.ftcPose.bearing;
                break;
            }
        }
        return angle;
    }

    private void initializeTagProcessor() {

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME));

        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
