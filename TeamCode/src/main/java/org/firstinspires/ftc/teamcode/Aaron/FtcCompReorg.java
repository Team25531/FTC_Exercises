package org.firstinspires.ftc.teamcode.Aaron;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp(name = "FtcCompReorg", group = "Aaron")
public class FtcCompReorg extends LinearOpMode {
    private ElapsedTime storageTimer = new ElapsedTime();
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotorEx outtake;
    private CRServo intake;
    private CRServo storageWheel;
    private AprilTagProcessor aprilTag;
    public static final String WEBCAM_NAME = "Webcam 1";
    public static final int TARGET_TAG_ID = 24;
    public static final int IDLE_VELOCITY = 600;
    private VisionPortal visionPortal;
    int goalVelocity = 0;
    double range = 0.05;
    double distanceToTarget = 0;
    double angleToTarget = 0;

    //boolean useOuttake = true;
    boolean isShooting = false;
    boolean isAutoAimEnabled = true;
    boolean isAtGoalVelocity = false;
    boolean shooterNeedsReset = false;
    boolean isAimedAtTarget = false;

    @Override
    public void runOpMode() throws InterruptedException {

        initializeMotors();
        initializeTagProcessor();
        SetIdleState();
        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            checkIfShooting();
            doDriving();
            autoAimOnOff();
            controlIntake();
            checkToResetState();
            checkSetIdleState();
            setGoalVelocity();
            runOuttakeMotor();
            doShooting();

            telemetry.update();
        }
    }

    private void doShooting() {
        if (!isShooting) return;

        //if we're using auto aim and it isn't yet at the target then let steering keep going.
        if (isAutoAimEnabled && !isAimedAtTarget) return;

        //if we are at goal, then run the storageWheel.
        if (isAtGoalVelocity && !shooterNeedsReset) {
            storageTimer.reset();
            storageWheel.setPower(-1);
            //todo: determine correct duration for this timer.
            while (opModeIsActive() & storageTimer.milliseconds() < 3000 && !shooterNeedsReset) {
                sleep(1);
                //todo: delete this telemetry.
                telemetry.addData("storeTimer", storageTimer.milliseconds());
                telemetry.update();
            }
            shooterNeedsReset = true;
            SetIdleState();
        }
        telemetry.addData("shooterNeedsReset", shooterNeedsReset);
    }

    private void setGoalVelocity() {
        //only compute velocity if we're actually shooting.
        if (isShooting && !shooterNeedsReset) {
            //todo: set these range values
            if (distanceToTarget > 55 && distanceToTarget < 129) {
                goalVelocity = (int) ((0.0006 * Math.pow(distanceToTarget, 2)) + (4.8385 * distanceToTarget) + 721.11);
                telemetry.addData("SHOOT, curDist", distanceToTarget);
            } else {
                telemetry.addData("NO SHOOT, curDist", distanceToTarget);
            }
        } else {
            SetIdleState();
        }
        telemetry.addData("goalVelocity", goalVelocity);
    }

    private void checkSetIdleState() {
        if (gamepad1.left_bumper) {
            SetIdleState();
        }
    }

    private void checkToResetState() {
        //Try to stop anything that isn't responding to other commands.
        if (gamepad1.right_bumper) {
            goalVelocity = 0;//but Idle is going to get reset after this.
            outtake.setPower(0);
            intake.setPower(0);
            storageWheel.setPower(0);
        }
    }

    private void controlIntake() {
        //Bring artifacts into the bot queue
        if (gamepad1.dpad_down) {
            intake.setPower(-1);
        }
        //Push artifacts out of the bot queue
        if (gamepad1.dpad_up) {
            intake.setPower(1);
        }
        //Stop intake with any other dpad or x.
        if (gamepad1.x || gamepad1.dpad_left || gamepad1.dpad_right) {
            intake.setPower(0);
        }
    }

    private void autoAimOnOff() {
        //toggle autoAim on off if something is wonky.
        if (gamepad1.yWasPressed()) {
            isAutoAimEnabled = !isAutoAimEnabled;
        }
        telemetry.addData("isAutoAimEnabled", isAutoAimEnabled);
    }

    private void checkIfShooting() {
        //to shoot, hold down the left_trigger.
        if (gamepad1.left_trigger > 0 && !shooterNeedsReset) {
            isShooting = true;
        } else if (gamepad1.left_trigger == 0) {
            isShooting = false;
            shooterNeedsReset = false;
        }
    }

    private void runOuttakeMotor() {
        double currentVelocity = outtake.getVelocity();
        double currentPower = outtake.getPower();
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

        outtake.setPower(currentPower);
    }

    private void SetIdleState() {
        goalVelocity = IDLE_VELOCITY; //min speed for Outtake wheel
        storageWheel.setPower(0); //stop the storage wheel
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

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

}
