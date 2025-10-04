package org.firstinspires.ftc.teamcode.neel;

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
//does all the imports

/*
https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
https://ftc-docs.firstinspires.org/apriltag-detection-values
RobotAutoDriveToAprilTagOmni and RobotAutoDriveToAprilTagTank

The only device needed in a hardware configuration is a Logi camera named
in the variable WEBCAM_NAME
 */

@SuppressLint("DefaultLocale")
@TeleOp(name = "AprilTagDetails", group = "Aaron")
//makes the group name and the name of the thing on the control hub
//@Disabled
public class copyOfAprilTagDetection extends LinearOpMode {
    public static final int GOAL_TAG_ID = 585;
    //makes a vairiable for the april tag that it wants to scan
    public static final String WEBCAM_NAME = "Webcam 1";
    // accesses the web cam that we want to use
    private AprilTagProcessor aprilTag;
    //creates an apriltag prossesr and names it
    private VisionPortal visionPortal;
    //creates a visionPortal and names it

    @Override
    public void runOpMode() {

        initializeTagProcessor();
        //adding the tagprossersor by initalizing it

        telemetry.addData(">", "Use scrcpy to view the camera output live");
        telemetry.addData(">", "Press START to start OpMode");
        telemetry.update();
        //adding telemetry to see if it works and udpates it so its on point

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                displayTagInfo();
                telemetry.addLine("--------------------------");
                //adds telemetry to make sure the displaying of the tag info works

                int range = getDistanceToTag(GOAL_TAG_ID);
                telemetry.addData("Value from getDistanceToTag", range);
                //adds telemetry to show the data from how far the april tag is away

                telemetry.update();
                //updates telemetry so its on point
            }
        }
        visionPortal.close();
        //closes that part of the code
    }

    private void initializeTagProcessor() {
        //explains the initializeTagProssor function
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        //its initiazizing everything by making it set to our prefrences to detect

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTag);
//acualy get the web cam with the hardware map
        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(aprilTag, true);
        //enables prosser to work and show its data
    }


    private void displayTagInfo() {
        //explains what the displaytagInfo funcion does

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        int countTagsFound = currentDetections.size();
//gets the size of the april tags
        if (countTagsFound == 0) {
            telemetry.addLine("Looking for tags");
            telemetry.addLine(String.format("Cannot Detect Goal Tag id %d", GOAL_TAG_ID));
            //if it can't detect april tags sends this error
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
                    //sends a bunch of data if it can see it relating to the distance it is from the april tag
                }
            }
        }

    }

    private int getDistanceToTag(int tagID) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        int range = 0;
        //gets all the apriltag detection

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == tagID) {
                range = (int) detection.ftcPose.range;
                break;
            }
        }
        return range;
        //gives you the info of the tag and the range of it
    }
}