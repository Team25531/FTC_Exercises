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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
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
import com.qualcomm.robotcore.util.Range;

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
                // --- BALL IS DETECTED ---
                ColorBlobLocatorProcessor.Blob largestBlob = blobs.get(0);
                Circle circleFit = largestBlob.getCircle();
                double currentRadius = circleFit.getRadius();
                double ballCenterX = circleFit.getX();

                // Center of the camera view (320 / 2 = 160)
                double cameraCenterX = 160.0;
                // How close to the center is "good enough" (in pixels)
                double centeringTolerance = 20.0;

                // Check if the ball is centered in the camera's view
                if (Math.abs(ballCenterX - cameraCenterX) > centeringTolerance) {
                    // --- STATE: CENTERING ON BALL ---
                    // The ball is visible but not in the center, so we need to turn.
                    telemetry.addLine("Ball detected. Centering...");

                    // A simple proportional turn. The farther the ball is from the center, the faster we turn.
                    double turnError = cameraCenterX - ballCenterX;
                    double kP_turn = 0.005; // Proportional constant for turning. Tune if needed.
                    double turnSpeed = turnError * kP_turn;

                    // Constrain the turn speed to a maximum
                    double maxTurnSpeed = 0.3;
                    turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

                    telemetry.addData("Ball Center X", "%.2f", ballCenterX);
                    telemetry.addData("Turn Speed", "%.2f", turnSpeed);


                    // Apply power to turn the robot. Negative speed turns left, positive turns right.
                    frontLeftMotor.setPower(-turnSpeed);
                    backLeftMotor.setPower(-turnSpeed);
                    frontRightMotor.setPower(turnSpeed);
                    backRightMotor.setPower(turnSpeed);

                } else {
                    // --- STATE: APPROACHING BALL ---
                    // The ball is centered, now we can drive towards it.
                    telemetry.addLine("Ball is centered. Approaching...");

                    // --- Driving Logic from your code ---
                    double targetRadius = 25.6; // <-- This is your calibrated stopping radius.
                    double driveSpeed = 0.2;
                    boolean shouldMove = currentRadius < targetRadius;

                    telemetry.addData("Current Radius", "%.2f", currentRadius);
                    telemetry.addData("Target Radius", "%.2f", targetRadius);
                    telemetry.addData("Should Move?", shouldMove);

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
                }
            } else {
                // --- STATE: SEARCHING FOR BALL ---
                telemetry.addLine("(No blobs found)");
                telemetry.addLine("Searching for ball... Turning right.");

                double turnSpeed = 0.3;

                // Turn the robot to the right to search
                frontLeftMotor.setPower(turnSpeed);
                backLeftMotor.setPower(turnSpeed);
                frontRightMotor.setPower(-turnSpeed);
                backRightMotor.setPower(-turnSpeed);
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