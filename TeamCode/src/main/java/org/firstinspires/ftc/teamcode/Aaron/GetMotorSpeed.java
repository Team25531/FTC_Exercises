package org.firstinspires.ftc.teamcode.Aaron;

import android.annotation.SuppressLint;
import android.util.Log;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
@TeleOp(name = "GetMotorSpeed", group = "Aaron")
@Disabled
public class GetMotorSpeed extends LinearOpMode {
    public static final int GOAL_TAG_ID = 22;
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
        //frontRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //  waitForStart();
        // intTagProcessor();

        telemetry.addData(">", "OpMode to display velocity of a motor.");
        telemetry.addData(">", "Press START to start OpMode");
//        telemetry.addData(">", "Use scrcpy to view the camera output live");
//        telemetry.addData(">", "Press START to start OpMode");
        telemetry.update();
        int targetA = 400;
        int targetY = 2000;
        int curGoal = 0;
        int range = 10;
        int ticksPerRev = 1440;
        waitForStart();
        double targetVelocity = 0;

        if (opModeIsActive()) {
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            runtime.reset();
            while (opModeIsActive()) {
                c.update();
                // motor.setVelocity(300);
                // runMotorAtTargetVelocity(targetVelocity);
                // telemetry.addData("cur velocity fun", getCurrentMotorVelocity());
                telemetry.addData("Curr Ticks/sec:", frontRightMotor.getVelocity());
                //    frontRightMotor.setVelocity(1440);
                //  telemetry.addData("cur velocity deg", frontRightMotor.getVelocity(AngleUnit.DEGREES));
                //    telemetry.addData("cur ticks", frontRightMotor.getCurrentPosition());
//                telemetry.addData(" ticks", frontRightMotor.);

//                double elapsedSeconds = runtime.seconds();
//                double ticksPerSecond = (frontRightMotor.getCurrentPosition() / elapsedSeconds) * 1;
//                telemetry.addData("Ticks/Sec", ticksPerSecond);
//
//                double RPM = frontRightMotor.getCurrentPosition() / (elapsedSeconds * ticksPerRev);
//                telemetry.addData("RPMs", RPM);
//                telemetry.addLine(String.format("RPM: %6.2f", RPM));

                double ratio = frontRightMotor.getVelocity() / ticksPerRev;
                telemetry.addLine(String.format("Curr RPM: %6.2f", ratio));

                if (c.B()) {
                    targetVelocity = 0;
                }
                if (c.A()) {
                    targetVelocity = targetA;
                }
                if (c.Y()) {
                    targetVelocity = targetY;
                }
//                if (c.dpadDownOnce()) targetVelocity -= .1;
//                if (c.dpadUpOnce()) targetVelocity += .1;

                //              targetVelocity = Range.clip(targetVelocity, -1, 1);

                telemetry.addData("targetVelocity", targetVelocity);

                frontRightMotor.setVelocity(targetVelocity);

                // frontRightMotor.setPower(targetVelocity);
//.3 250
//.4 340
//.5 425

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