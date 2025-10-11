package org.firstinspires.ftc.teamcode.Santiago;

import android.util.Log;
import android.util.Size;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Aaron.Controller;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.videoio.Videoio;

@TeleOp(group = "Santiago", name = "GetMotorSpeedSantiago")
public class GetMotorSpeedSantiago extends LinearOpMode {
    public static final int GOAL_TAG_ID = 585;
    public static final String WEBCAM_NAME = "Webcam 1";
    private AprilTagProcessor aprilTag;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;
    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private ElapsedTime runtime = new ElapsedTime();
    private VisionPortal visionPortal;

    public void runOpMode() {
        Controller c = new Controller(this.gamepad1);
        this.backLeftMotor = (DcMotorEx) this.hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        this.backRightMotor = (DcMotorEx) this.hardwareMap.get(DcMotorEx.class, "backRightMotor");
        this.frontLeftMotor = (DcMotorEx) this.hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        this.frontRightMotor = (DcMotorEx) this.hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        double targetTicks = LynxServoController.apiPositionFirst;
        this.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.telemetry.addData(">", (Object) "OpMode to display velocity of a motor.");
        this.telemetry.addData(">", (Object) "Press START to start OpMode");
        this.telemetry.update();
        waitForStart();
        double targetVelocity = 0.3d;
        if (opModeIsActive()) {
            this.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.runtime.reset();
            while (true) {
                double targetVelocity2 = targetVelocity;
                if (opModeIsActive()) {
                    c.update();
                    this.telemetry.addData("cur velocity fun", (Object) Double.valueOf(getCurrentMotorVelocity()));
                    this.telemetry.addData("cur velocity mot", (Object) Double.valueOf(this.frontRightMotor.getVelocity()));
                    this.telemetry.addData("cur velocity deg", (Object) Double.valueOf(this.frontRightMotor.getVelocity(AngleUnit.DEGREES)));
                    this.telemetry.addData("cur ticks", (Object) Integer.valueOf(this.frontRightMotor.getCurrentPosition()));
                    this.telemetry.addData("RPM", (Object) Double.valueOf((((double) this.frontRightMotor.getCurrentPosition()) / this.runtime.seconds()) * 1.0d));
                    if (this.gamepad1.a) {
                        targetTicks = (double) 400;
                    }
                    if (this.gamepad1.y) {
                        targetTicks = (double) Videoio.CAP_OPENCV_MJPEG;
                    }
                    if (this.gamepad1.b) {
                        targetTicks = LynxServoController.apiPositionFirst;
                    }
                    if (((double) 0) > targetTicks) {
                        double curPower = this.frontRightMotor.getPower() - 0.05d;
                    }
                    if (((double) 0) < targetTicks) {
                        double curPower2 = this.frontRightMotor.getPower() + 0.05d;
                    }
                    targetVelocity = Range.clip(targetVelocity2, -1.0d, 1.0d);
                    this.telemetry.addData("targetVelocity", (Object) Double.valueOf(targetVelocity));
                    this.frontRightMotor.setPower(targetVelocity);
                    this.telemetry.update();
                } else {
                    return;
                }
            }
        }
    }

    public double getCurrentMotorVelocity() {
        return this.frontRightMotor.getVelocity(AngleUnit.DEGREES);
    }

    public void runMotorAtTargetVelocity(double targetVelocityDegreesPerSec) {
        double currentVelocity = getCurrentMotorVelocity();
        double motorPower = Range.clip((targetVelocityDegreesPerSec - currentVelocity) * 0.005d, -1.0d, 1.0d);
        if (motorPower < LynxServoController.apiPositionFirst) {
            double curPower = this.frontRightMotor.getPower();
            motorPower = curPower - (curPower * 0.005d);
        }
        this.telemetry.addData("power", (Object) Double.valueOf(motorPower));
        Log.d("power::", String.valueOf(motorPower) + " - " + ((int) currentVelocity));
        this.frontRightMotor.setPower(motorPower);
        double clip = Range.clip(motorPower, -1.0d, 1.0d);
    }

    private void intTagProcessor() {
        this.aprilTag = new AprilTagProcessor.Builder().setDrawTagOutline(true).setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES).build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera((CameraName) this.hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, Videoio.CAP_PROP_XI_CC_MATRIX_01));
        builder.addProcessor(this.aprilTag);
        this.visionPortal = builder.build();
        this.visionPortal.setProcessorEnabled(this.aprilTag, true);
    }

    private void displayTagInfo() {
        List<AprilTagDetection> currentDetections = this.aprilTag.getDetections();
        int countTagsFound = currentDetections.size();
        if (countTagsFound == 0) {
            this.telemetry.addLine("Looking for tags");
            this.telemetry.addLine(String.format("Cannot Detect Goal Tag id %d", new Object[]{585}));
            return;
        }
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 585) {
                this.telemetry.addLine(String.format("Range (inches)\t %6.1f", new Object[]{Double.valueOf(detection.ftcPose.range)}));
                this.telemetry.addLine(String.format("Bearing (deg) \t %6.1f", new Object[]{Double.valueOf(detection.ftcPose.bearing)}));
                this.telemetry.addLine("Tag Position:");
                this.telemetry.addLine("\tPositive is Left of center");
                this.telemetry.addLine("\tNegative is Right of center");
                this.telemetry.addLine(String.format("Tag Name: %s \t", new Object[]{detection.metadata.name}));
                this.telemetry.addLine(String.format("Tag ID: %d \t", new Object[]{Integer.valueOf(detection.metadata.id)}));
                this.telemetry.addData("Total tag count in view", (Object) Integer.valueOf(countTagsFound));
            }
        }
    }

    private int getDistanceToTag(int tagID) {
        int range = 0;
        for (AprilTagDetection detection : this.aprilTag.getDetections()) {
            if (detection.id == tagID) {
                range = (int) detection.ftcPose.range;
            }
        }
        return range;
    }
}
    