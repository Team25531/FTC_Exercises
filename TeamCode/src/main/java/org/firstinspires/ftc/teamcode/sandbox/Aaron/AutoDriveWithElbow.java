package org.firstinspires.ftc.teamcode.sandbox.Aaron;

import android.annotation.SuppressLint;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous(name = "AutoDriveWithElbow", group = "auto")
@Disabled
public class AutoDriveWithElbow extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();

    public static Servo wristLeft;
    public static Servo wristRight;
    private int tickTarget = 100;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private IMU imu = null;      // Control/Expansion Hub IMU
    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;
    private AHRS navx_device;
    GlobalState globalState;


    private double turnSpeed = 0;
    private double headingError = 0;
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable.
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable.
    static final double HEADING_THRESHOLD = 1.0;    // How close must the heading get to the target before moving to next step.


    @Override
    public void runOpMode() throws InterruptedException {

        frontLeftMotor = hardwareMap.dcMotor.get("m2");//2
        backLeftMotor = hardwareMap.dcMotor.get("m0");//0
        frontRightMotor = hardwareMap.dcMotor.get("m3");//3
        backRightMotor = hardwareMap.dcMotor.get("m1");//1
        wristLeft = hardwareMap.servo.get("wristLeft");
      //  wristRight = hardwareMap.servo.get("wristRight");

        globalState = new GlobalState();
        globalState.elbow = backRightMotor;

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData);
        while (navx_device.isCalibrating()) {
            Thread.sleep(50);
        }
        gyro = (IntegratingGyroscope) navxMicro;

        navxMicro.initialize();
        // If you're only interested int the IntegratingGyroscope interface, the following will suffice.
        // gyro = hardwareMap.get(IntegratingGyroscope.class, "navx");

        // The gyro automatically starts calibrating. This takes a few seconds.
        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        // Wait until the gyro calibration is complete
        timer.reset();
        while (navxMicro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sendTelemetry(true);
            Thread.sleep(50);
        }
        telemetry.log().add("Gyro Calibrated. Press Start.");

        while (opModeInInit()) {
            sendTelemetry(true);
        }

        //waiting for the Play button to start the anon op
        waitForStart();

        //this only needs to be called once, after that calling SetElbowPosition will move the arm
        StartElbowControl();

        //set the Tick position of the elbow
        SetElbowPosition(-400);
        double driveSpeed;

        driveSpeed = 0.6;

        //always starts from zero. Left is positive, Right is negative.
        turnToHeading(driveSpeed, 45);
        //holding can allow it to stabilize on a critical heading. Not required, but improves accuracy.
        holdHeading(driveSpeed, 45, 1);
        //this sets the current forward direction of the robot to zero, so the next turn command starts from zero.
        resetHeadingIMU();
        sleep(10);


        //move a specific distance in ticks. It uses the average of the 4 wheel ticks
        //use a positive number for Forward, and a negative number for Reverse.
        MoveStraightTicks(1000, driveSpeed);

        //sometimes a little sleep time helps things from being to jittery.
        sleep(10);

        turnToHeading(driveSpeed, -45);
        holdHeading(driveSpeed, -45, 1);

        sleep(10);

        MoveStraightTicks(-2000, driveSpeed);
        sleep(10);

        turnToHeading(driveSpeed, 75);
        sleep(10);


        SetElbowPosition(-200);
        sleep(10);


        //only needs to be called once at end of operation
        StopElbowControl();
        requestOpModeStop();
    }

    private void StopElbowControl() {
        globalState.isCancelled = true;
    }

    private void StartElbowControl() {
        new Thread(new BackgroundElbowMove(globalState)).start();
    }

    private void SetElbowPosition(int position) {
        globalState.setElbowPosition(position);

    }

    public int getAverageTicksMoved() {

        int total = frontLeftMotor.getCurrentPosition() +
                frontRightMotor.getCurrentPosition() +
                backLeftMotor.getCurrentPosition() +
                backRightMotor.getCurrentPosition();
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


    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            //  telemetry.addData("Target Pos L:R", "%7d:%7d", leftTarget, rightTarget);
            telemetry.addData("Actual Pos L:R", "%7d:%7d", frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        int frontLeftMotorTicks = frontLeftMotor.getCurrentPosition();
        int backLeftMotorTicks = backLeftMotor.getCurrentPosition();
        int frontRightMotorTicks = frontRightMotor.getCurrentPosition();
        int backRightMotorTicks = backRightMotor.getCurrentPosition();
        int averageTicks = (frontLeftMotorTicks + backLeftMotorTicks + frontRightMotorTicks + backRightMotorTicks) / 4;
        telemetry.addData("> heading IMU", "%4.1f", getHeadingIMU())
                .addData(">heading NavX", "%4.1f", getHeadingNavX())
                .addData(">goal heading ", "%4.1f", headingStore)
                .addData("> delta heading", "%4.1f", headingError)
                .addData("> turn speed", "%4.1f", turnSpeed)
                .addData(">frontLeftMotor ticks", frontLeftMotorTicks)
                .addData(">backLeftMotor ticks", backLeftMotorTicks)
                .addData(">frontRightMotor ticks", frontRightMotorTicks)
                .addData(">frontLeftMotor ticks", backRightMotorTicks)
                .addData(">Average Motor ticks", averageTicks)
                .addData("> Left Servo Pos", wristLeft.getPosition())
                .addData(">Right Servo Pos", wristRight.getPosition())
                .addData("> Elbow State", globalState.getElbowPosition());
        telemetry.update();
    }

    public double getHeadingIMU() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public double getHeadingNavX() {
//        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        return angles.firstAngle;
        return navx_device.getYaw();
    }

    public void resetHeadingIMU() {
        imu.resetYaw();
    }


    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    @SuppressLint("DefaultLocale")
    String formatDegrees(double degrees) {
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
