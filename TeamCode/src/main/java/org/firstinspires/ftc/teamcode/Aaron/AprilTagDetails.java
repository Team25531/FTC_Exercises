package org.firstinspires.ftc.teamcode.Aaron;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
https://ftc-docs.firstinspires.org/apriltag-detection-values
RobotAutoDriveToAprilTagOmni and RobotAutoDriveToAprilTagTank

The only device needed in a hardware configuration is a Logi camera named
in the variable WEBCAM_NAME
 */

@SuppressLint("DefaultLocale")
@TeleOp(name = "AprilTagDetails", group = "Aaron")
//@Disabled
public class AprilTagDetails extends LinearOpMode {
    public static final int GOAL_TAG_ID = 22;
    public static final String WEBCAM_NAME = "Webcam 1";

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        initializeTagProcessor();

        telemetry.addData(">", "Use scrcpy to view the camera output live");
        telemetry.addData(">", "Press START to start OpMode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                displayTagInfo();
                telemetry.addLine("--------------------------");

                int range = getDistanceToTag(GOAL_TAG_ID);
                telemetry.addData("Value from getDistanceToTag", range);

                telemetry.update();
            }
        }
        visionPortal.close();
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
                break;
            }
        }
        return range;
    }
}