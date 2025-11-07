package org.firstinspires.ftc.teamcode.neel;


import android.annotation.SuppressLint;
import android.util.Log;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Aaron.Controller;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@SuppressLint("DefaultLocale")
@TeleOp(name = "NeelGetMotorSpeed", group = "Neel")
@Disabled
//@Disabled
public class getMotorSpeed extends LinearOpMode {
    public static final int GOAL_TAG_ID = 24;
    public static final String WEBCAM_NAME = "Webcam 1";

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;
    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;

    // private DcMotorEx motor;
    //     frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
//        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        Controller c = new Controller(gamepad1);
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //motor = frontRightMotor;

        //  motor = hardwareMap.get(DcMotorEx.class, "fr");

        // use braking to slow the motor down faster
        frontRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // disables the default velocity control
        // this does NOT disable the encoder from counting,
        // but lets us simply send raw motor power.
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //  waitForStart();
        // intTagProcessor();
        intTagProcessor();
        telemetry.addData(">", "OpMode to display velocity of a motor.");
        telemetry.addData(">", "Press START to start OpMode");
//        telemetry.addData(">", "Use scrcpy to view the camera output live");
//        telemetry.addData(">", "Press START to start OpMode");
        telemetry.update();
        int targetY = 700;
        int targetA = 1440;
        int curTarget = 0;
        double Minrange = 0;
        double Maxrange = 0;
        double range = 0.05;
        waitForStart();
        double targetVelocity = .6;

        if (opModeIsActive()) {
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            runtime.reset();
            while (opModeIsActive()) {
                c.update();
                // motor.setVelocity(300);
                // runMotorAtTargetVelocity(targetVelocity);
                telemetry.addData("cur velocity fun", getCurrentMotorVelocity());
                telemetry.addData("cur velocity mot", frontRightMotor.getVelocity());
                telemetry.addData("cur velocity deg", frontRightMotor.getVelocity(AngleUnit.DEGREES));
                telemetry.addData("cur ticks", frontRightMotor.getCurrentPosition());
//                telemetry.addData(" ticks", frontRightMotor.);

                double elapsedSeconds = runtime.seconds();
                double rpm = (frontRightMotor.getCurrentPosition() / elapsedSeconds) * 1;

                telemetry.addData("RPM", rpm);

                if (c.dpadDownOnce()) targetVelocity -= .1;
                if (c.dpadUpOnce()) targetVelocity += .1;
                targetVelocity = Range.clip(targetVelocity, -1, 1);
                telemetry.addData("targetVelocity", targetVelocity);
                //frontRightMotor.setPower(targetVelocity);
//.3 250
                displayTagInfo();
                double rangeToTarget = getDistanceToTag(GOAL_TAG_ID);

                double angleToTarget = getAngleToTag(GOAL_TAG_ID);
                telemetry.addData("Current distance",rangeToTarget);
                telemetry.addData("current AngelToTarget", angleToTarget);
                curTarget = 0;
                if (rangeToTarget < 76){
                    curTarget = targetY;
                }

                if (rangeToTarget > 100){
                    curTarget = targetA;
                }

                if (gamepad1.a) {
                    curTarget = targetA;
                    telemetry.addData("motorTargetSetTo1", curTarget);
                }
//.5 425
                if (gamepad1.b) {
                    curTarget = 0;
                    telemetry.addData("motorTargetSetTo12", curTarget);

                }
                if (gamepad1.y) {
                    curTarget = targetY;
                    telemetry.addData("motorTargetSetTo123", curTarget);
                }

                telemetry.addData("motorTargetSetTo", curTarget);
                double currentVelocity = frontRightMotor.getVelocity();
                Minrange = curTarget -(curTarget* range);
                Maxrange = curTarget +(curTarget * range);
                if (currentVelocity < Minrange) {
                    double currentPowerMore = frontRightMotor.getPower();
                    currentPowerMore = currentPowerMore + 0.001;
                    frontRightMotor.setPower(currentPowerMore);
                    telemetry.addData("Works!", 0);


                }
                if (currentVelocity > Maxrange) {
                    double currentPowerLess = frontRightMotor.getPower();
                    currentPowerLess = currentPowerLess - 0.001;
                    frontRightMotor.setPower(currentPowerLess);
                    telemetry.addData("Works1234!", 0);
                }
                if (currentVelocity < Maxrange && currentVelocity > Minrange) {
                    telemetry.addData("SHOOTNOW!", 0);
                }
//                displayTagInfo();
//                telemetry.addLine("--------------------------");
//
//                int range = getDistanceToTag(GOAL_TAG_ID);
//                telemetry.addData("Value from getDistanceToTag", range);

                // telemetry.setAutoClear(false);
                telemetry.update();
            }
        }
//        visionPortal.close();
    }


//    public double getCurrentMotorVelocity() {
//        // Get velocity in degrees per second
//        double velocity = motor.getVelocity(AngleUnit.DEGREES);
//        return velocity;
//    }

    public double getCurrentMotorVelocity() {
        // Get velocity in degrees per second
        double velocity = frontRightMotor.getVelocity(AngleUnit.DEGREES);
        return velocity;
    }

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

    public void runMotorAtTargetVelocity(double targetVelocityDegreesPerSec) {
        // Example: simple proportional control (for illustration, a full PID is recommended)
        double currentVelocity = getCurrentMotorVelocity();
        //  double error = Math.abs( targetVelocityDegreesPerSec - currentVelocity);
        double error = targetVelocityDegreesPerSec - currentVelocity;

        // Adjust motor power based on error (this is a very basic example)
        double kP = 0.005; // Proportional gain
//        double motorPower = Math.max(Math.min((error * kP), 1), -1);
        double motorPower = Range.clip((error * kP), -1, 1);
        if (motorPower < 0) {
            double curPower = frontRightMotor.getPower();
            double newPower = curPower - (curPower * kP);
            motorPower = newPower;
        }
        telemetry.addData("power", motorPower);
        Log.d("power::", String.valueOf(motorPower) + " - " + (int) currentVelocity);

        frontRightMotor.setPower(motorPower);
        double foo = Range.clip(motorPower, -1, 1);
    }


    private void intTagProcessor() {

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


    private void displayTagInfo() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        int countTagsFound = currentDetections.size();

        if (countTagsFound == 0) {
            telemetry.addLine("Looking for tags");
            telemetry.addLine(String.format("Cannot Detect Goal Tag id %d", GOAL_TAG_ID));
        } else {
            for (AprilTagDetection detection : currentDetections) {
                int id = detection.id;
                if (id == GOAL_TAG_ID) {
                    telemetry.addLine(String.format("Range (inches)\t %6.1f", detection.ftcPose.range));
                    telemetry.addLine(String.format("Bearing (deg) \t %6.1f", detection.ftcPose.bearing));
                    telemetry.addLine("Tag Position:");
                    telemetry.addLine("\tPositive is Left of center");
                    telemetry.addLine("\tNegative is Right of center");
                    telemetry.addLine(String.format("Tag Name: %s \t", detection.metadata.name));
                    telemetry.addLine(String.format("Tag ID: %d \t", detection.metadata.id));
                    telemetry.addData("Total tag count in view", countTagsFound);
                }
            }
        }

    }
    private int getAngleToTag(int tagID) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        int range = 0;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == tagID) {
                range = (int) detection.ftcPose.bearing;
                break;
            }
        }
        return range;
    }


    private int getDistanceToTag(int tagID) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        int range = 0;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == tagID) {
                range = (int) detection.ftcPose.range;
            }
        }
        return range;
    }
}