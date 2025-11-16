/*
 * Copyright (c) 2024 Phil Malone
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is

 * furnished to do so, to a certain condition.
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.FTCDecodeComp;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

@TeleOp(name = "Auto Test", group = "FTCComp")
public class AutoTest extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotorEx outtake;
    private CRServo intake;
    private CRServo storageWheel;

    @Override
    public void runOpMode() {
        ColorBlobLocatorProcessor colorLocator = initBallFinderCamera();
        initializeMotors();

        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // Wait for the user to press start on the Driver Station.
        waitForStart();

        while (opModeIsActive()) {
            // Read the current list of detected blobs
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            // Filter out blobs that are too small/large or not circular enough.
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    25, 20000, blobs);

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                    0.6, 1, blobs);

            telemetry.addLine("Circularity Radius Center");

            // Check if any blobs were found after filtering
            if (!blobs.isEmpty()) {
                // The blobs are sorted by area by default, so the first one is the largest.
                ColorBlobLocatorProcessor.Blob largestBlob = blobs.get(0);
                Circle circleFit = largestBlob.getCircle();
                double currentRadius = circleFit.getRadius();

                // Calculate the distance to the largest blob for telemetry purposes only.
                double distance = findDistance((float) currentRadius);

                // Display info for the largest blob
                telemetry.addLine(String.format("%5.3f      %3.0f     (%3.0f,%3.0f)",
                        largestBlob.getCircularity(), currentRadius, circleFit.getX(), circleFit.getY()));
                telemetry.addData("Calculated Distance:", "%.2f inches", distance);

                // --- Driving Logic ---
                // STEP 1: Find this value by running the OpMode and placing the robot
                // at the desired distance. Read the "Current Radius" from telemetry
                // and put that number here.
                double targetRadius = 25.6; // <-- REPLACE THIS WITH YOUR CALIBRATED VALUE

                // Using a slower speed to prevent overshooting the target.
                double driveSpeed = 0.2;

                boolean shouldMove = currentRadius < targetRadius;

                // Add diagnostic telemetry to see the decision-making process
                telemetry.addData("Current Radius", "%.2f", currentRadius);
                telemetry.addData("Target Radius", "%.2f", targetRadius);
                telemetry.addData("Should Move?", shouldMove);

                // Drive forward if the ball's radius is smaller than our target radius.
                // Otherwise, stop.
                if (shouldMove) {
                    // Drive forward
                    frontRightMotor.setPower(driveSpeed);
                    frontLeftMotor.setPower(driveSpeed);
                    backRightMotor.setPower(driveSpeed);
                    backLeftMotor.setPower(driveSpeed);
                } else {
                    // Stop if we are close enough
                    frontRightMotor.setPower(0);
                    frontLeftMotor.setPower(0);
                    backRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                }

            } else {
                telemetry.addLine("(No blobs found)");
                telemetry.addLine("Robot will not move if it can't see the ball.");
                // Stop if no ball is detected
                frontRightMotor.setPower(0);
                frontLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                backLeftMotor.setPower(0);
            }

            telemetry.update();
            sleep(20); // Small pause to prevent spamming commands
        }
    }

    private ColorBlobLocatorProcessor initBallFinderCamera() {
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
//                .setRoi(ImageRegion.entireFrame())
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(55, 55, 255)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image
                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();



        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        return colorLocator;
    }


    private double findDistance(float radius) {
        double distance = 0;
        distance = (0.0382*radius*radius) + (4.0427*radius) - 127.07;
        return distance;
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
}