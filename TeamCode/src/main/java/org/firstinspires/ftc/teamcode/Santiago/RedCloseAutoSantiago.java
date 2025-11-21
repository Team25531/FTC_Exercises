package org.firstinspires.ftc.teamcode.Santiago;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

@Autonomous(name = " Red FTC Comp Close santiago", group = " Red Ftc Comp santiago")
@Disabled
public class RedCloseAutoSantiago extends LinearOpMode {
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
    public static int IDLE_VELOCITY = 600;
    private VisionPortal visionPortal;
    int goalVelocity = 0;
    double range = 0.05;
    double distanceToTarget = 0;
    double angleToTarget = 0;
    double currentVelocity = 0;
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

//            double ve;
//
//            ve = (int) ((-0.0861 * Math.pow(distanceToTarget, 2)) + (20.729 * distanceToTarget) + 104.51);
//            telemetry.addData("current velocity", ve);
//
//
//            if (isStopRequested()) return;
//            while (distanceToTarget <= 55) {
//                frontLeftMotor.setPower(-0.2);
//                frontRightMotor.setPower(-0.2);
//                backRightMotor.setPower(-0.2);
//                backLeftMotor.setPower(-0.2);
//                setGoalVelocity();
//                distanceToTarget = getDistanceToTag(24);
//
//            }
//            distanceToTarget = getDistanceToTag(24);
//            shooterNeedsReset = false;
//
//            if (distanceToTarget >= 55) {
//
//
//                frontLeftMotor.setPower(0);
//                frontRightMotor.setPower(0);
//                backRightMotor.setPower(0);
//                backLeftMotor.setPower(0);
//                intake.setPower(-1);
//                setGoalVelocity();
//                runOuttakeMotor();
//                checkIfShooting();
//
//                doShooting();
//
//                if (shooterNeedsReset) {
//                    distanceToTarget = getDistanceToTag(24);
//                    while (distanceToTarget <= 97 && distanceToTarget >= 30) {
//                        frontLeftMotor.setPower(0.3);
//                        frontRightMotor.setPower(-0.2);
//                        backRightMotor.setPower(0.3);
//                        backLeftMotor.setPower(-0.2);
//                        distanceToTarget = getDistanceToTag(24);
//                    }
//                    distanceToTarget = getDistanceToTag(24);
//                    resetRuntime();
//                    while (distanceToTarget == 0) {
//
//                        double Runtime = getRuntime();
//                        if(Runtime< 2){
//                            frontLeftMotor.setPower(0.3);
//                            frontRightMotor.setPower(-0.2);
//                            backRightMotor.setPower(0.3);
//                            backLeftMotor.setPower(-0.2);
//
//
//                        }
//                        if(Runtime>1) {
//                            frontLeftMotor.setPower(0);
//                            frontRightMotor.setPower(0);
//                            backRightMotor.setPower(0);
//                            backLeftMotor.setPower(0);
//                            outtake.setPower(0);
//                        }
//                        distanceToTarget = getDistanceToTag(24);
//                    }
//                }
//
//
//            }
            telemetry.addData("preview on/off", "... Camera Stream\n");
            ColorBlobLocatorProcessor colorLocator = initBallFinderCamera();
            // Read the current list
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            /*
             * The list of Blobs can be filtered to remove unwanted Blobs.
             *   Note:  All contours will be still displayed on the Stream Preview, but only those
             *          that satisfy the filter conditions will remain in the current list of
             *          "blobs".  Multiple filters may be used.
             *
             * To perform a filter
             *   ColorBlobLocatorProcessor.Util.filterByCriteria(criteria, minValue, maxValue, blobs);
             *
             * The following criteria are currently supported.
             *
             * ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA
             *   A Blob's area is the number of pixels contained within the Contour.  Filter out any
             *   that are too big or small. Start with a large range and then refine the range based
             *   on the likely size of the desired object in the viewfinder.
             *
             * ColorBlobLocatorProcessor.BlobCriteria.BY_DENSITY
             *   A blob's density is an indication of how "full" the contour is.
             *   If you put a rubber band around the contour you would get the "Convex Hull" of the
             *   contour. The density is the ratio of Contour-area to Convex Hull-area.
             *
             * ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO
             *   A blob's Aspect ratio is the ratio of boxFit long side to short side.
             *   A perfect Square has an aspect ratio of 1.  All others are > 1
             *
             * ColorBlobLocatorProcessor.BlobCriteria.BY_ARC_LENGTH
             *   A blob's arc length is the perimeter of the blob.
             *   This can be used in conjunction with an area filter to detect oddly shaped blobs.
             *
             * ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY
             *   A blob's circularity is how circular it is based on the known area and arc length.
             *   A perfect circle has a circularity of 1.  All others are < 1
             */
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    50, 20000, blobs);  // filter out very small blobs.

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                    0.6, 1, blobs);
            /* filter out non-circular blobs.
             * NOTE: You may want to adjust the minimum value depending on your use case.
             * Circularity values will be affected by shadows, and will therefore vary based
             * on the location of the camera on your robot and venue lighting. It is strongly
             * encouraged to test your vision on the competition field if your event allows
             * sensor calibration time.
             */

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_DENSITY, 0.3, 1, blobs
            );

            /*
             * The list of Blobs can be sorted using the same Blob attributes as listed above.
             * No more than one sort call should be made.  Sorting can use ascending or descending order.
             * Here is an example.:
             *   ColorBlobLocatorProcessor.Util.sortByCriteria(
             *      ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs);
             */

//            telemetry.addLine("Circularity Radius Center");

            // Display the Blob's circularity, and the size (radius) and center location of its circleFit.
            for (ColorBlobLocatorProcessor.Blob b : blobs) {

                Circle circleFit = b.getCircle();
                if (circleFit.getY() >= 175) {
                    telemetry.clearAll();
                }
                else {
                    if (circleFit.getX()<=145 ) {
                        telemetry.addLine("Twisting Left");
                        telemetry.update();
                        microTwistLeft();
                    }
                    else if (circleFit.getX()>145&&circleFit.getX()<175) {
                        //go forward until y > 175
                        //turn on intake and pick up the ball
                    }
                    else {
                        telemetry.addLine("Twisting Right");
                        telemetry.update();
                        microTwistRight();
                    }



                    //if x between
                    telemetry.addLine(String.format("%5.3f    D%5.3f    R%3d    (%3d,%3d)",
                            b.getCircularity(), b.getDensity(), (int) circleFit.getRadius(), (int) circleFit.getX(), (int) circleFit.getY()));
                    telemetry.addLine("Distance: " + findDistance(circleFit.getRadius()));
                }
            }
            telemetry.update();
            //y=0.0382x^2 +4.0427x-127.07

//            telemetry.update();


            sleep(100); // Match the telemetry update interval.

            telemetry.update();
        }
    }

    private void doShooting() {
        //if we are at goal, then run the storageWheel.
        if (isAtGoalVelocity) {
            storageTimer.reset();
            storageWheel.setPower(-1);
            //todo: determine correct duration for this timer.
            while (opModeIsActive() & storageTimer.milliseconds() < 4000 && !shooterNeedsReset) {
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
    private ColorBlobLocatorProcessor initBallFinderCamera() {
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                //.setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
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
                .setLiveViewContainerId(0)

                .build();

        return colorLocator;
    }
    private double findDistance(float radius) {
        double distance = 0;
        distance = 0.0071*radius*radius - 1.3406*radius + 73.781;
        return distance;
    }
    private void microTwistRight() {
        frontLeftMotor.setPower(0.1);
        backLeftMotor.setPower(0.1);
        frontRightMotor.setPower(-0.1);
        backRightMotor.setPower(-0.1);
        sleep(100);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    private void microTwistLeft() {
        frontLeftMotor.setPower(-0.1);
        backLeftMotor.setPower(-0.1);
        frontRightMotor.setPower(0.1);
        backRightMotor.setPower(0.1);
        sleep(100);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
