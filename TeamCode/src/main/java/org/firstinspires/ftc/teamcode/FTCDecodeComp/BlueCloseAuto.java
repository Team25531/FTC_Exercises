package org.firstinspires.ftc.teamcode.FTCDecodeComp;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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


@Autonomous(name = " Red FTC Comp Close", group = " Red Ftc Comp")
public class BlueCloseAuto extends LinearOpMode {
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
    public static final int TARGET_TAG_ID = 20;
    public static int IDLE_VELOCITY = 600;
    private VisionPortal visionPortal;
    int goalVelocity = 0;
    double range = 0.05;
    double distanceToTarget = 0;
    double angleToTarget = 0;
    double currentVelocity=0;
    double currentPower = 0;
    boolean isIdleEnabled = false;

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

        waitForStart();


        while (opModeIsActive()) {

            double ve;

            ve = (int) ((-0.0861 * Math.pow(distanceToTarget, 2)) + (20.729 * distanceToTarget) + 104.51);
            telemetry.addData("current velocity", ve);


            if (isStopRequested()) return;
            while(distanceToTarget <= 55){
                frontLeftMotor.setPower(-0.1);
                frontRightMotor.setPower(-0.1);
                backRightMotor.setPower(-0.1);
                backLeftMotor.setPower(-0.1);
                setGoalVelocity();
                distanceToTarget = getDistanceToTag(20);

            }
            distanceToTarget = getDistanceToTag(20);
            shooterNeedsReset = false;

            if(distanceToTarget >= 55){
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                backLeftMotor.setPower(0);

                setGoalVelocity();
                runOuttakeMotor();
                checkIfShooting();

                doShooting();
                distanceToTarget = getDistanceToTag(24);


            }

            telemetry.update();
        }
    }

    private void doShooting() {
        //if we are at goal, then run the storageWheel.
        if (isAtGoalVelocity) {
            storageTimer.reset();
            storageWheel.setPower(-1);
            //todo: determine correct duration for this timer.
            while (opModeIsActive() & storageTimer.milliseconds() < 3000 && !shooterNeedsReset) {
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
            tempVelocity = (int) (693.198761 + 1191.999926 * (1.0 - Math.exp(-0.007992 * distanceToTarget)));
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
