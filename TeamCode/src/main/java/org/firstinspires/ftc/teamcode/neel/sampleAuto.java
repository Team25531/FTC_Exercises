package org.firstinspires.ftc.teamcode.neel;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Aaron.y2024.BackgroundElbowMove;
import org.firstinspires.ftc.teamcode.Aaron.y2024.GlobalState;

import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
@Disabled
@Autonomous(name = "autosample", group = "auto")
//we need to add the DcMotors
public class sampleAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor;// = hardwareMap.dcMotor.get("frontLeft");
    private DcMotor backLeftMotor;// = hardwareMap.dcMotor.get("backLeft");
    private DcMotor frontRightMotor;// = hardwareMap.dcMotor.get("frontRight");
    private DcMotor backRightMotor;// = hardwareMap.dcMotor.get("backRight");
    private DcMotor extension;
    public static DcMotor elbow;// = hardwareMap.dcMotor.get("backRight");
    private CRServo finger;
    private static int elbowPosition = -200;

    double position = 0.312;
    GlobalState globalState = new GlobalState();
    private int tickTarget = 100;

    private double targetHeading = 0;
    private double driveSpeed = 0.3;
    private double turnSpeed = 0;
    private double headingError = 0;
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable.
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable.
    static final double HEADING_THRESHOLD = 1.0;    // How close must the heading get to the target before moving to next step.

    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private int leftTarget = 0;
    private int rightTarget = 0;
    int COUNTS_PER_INCH = 115;

    final double ARM_TICKS_PER_DEGREE = 28 // number of encoder ticks per rotation of the bare motor
            * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
            * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
            * 1 / 360.0; // we want ticks per degree, not per rotation


    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    private IMU imu = null;

    private void forward (DcMotor fL, DcMotor bL, DcMotor fR, DcMotor bR, double time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path 11", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            fL.setPower(0.5);
            fR.setPower(0.5);
            bL.setPower(0.5);
            bR.setPower(0.5);

        }
        runtime.reset();

    }

    private void straightLine(DcMotor fL, DcMotor bL, DcMotor fR, DcMotor bR, double time) {
        runtime.reset();
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path 11", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            fL.setPower(0.5);
            fR.setPower(0.5);
            bL.setPower(0.5);
            bR.setPower(0.5);

        }
        runtime.reset();


    }


    @Override
    public void runOpMode() throws InterruptedException {

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        extension = hardwareMap.dcMotor.get("extension");

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        extension.setDirection(DcMotor.Direction.REVERSE);

//      commenting out original direction.
//        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
//        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotor.Direction.FORWARD);


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elbow = hardwareMap.dcMotor.get("elbow");
        finger = hardwareMap.crservo.get("finger");


        // elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setDirection(DcMotor.Direction.REVERSE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


}