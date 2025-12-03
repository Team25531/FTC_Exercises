package org.firstinspires.ftc.teamcode.neel;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@TeleOp(name = "Flipper", group = "Flipper")
public class flipperTest extends LinearOpMode {

    public static double NEW_P = 7.0;
    public static double NEW_I = 0.0;
    public static double NEW_D = 0.0;
    public static double NEW_F = 15.0;

    private CRServo intake2;
    private Servo flipper;
    private DcMotorEx outtake;
    private CRServo intake;
    private CRServo intake3;

    private VoltageSensor myControlHubVoltageSensor;


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
    boolean isFeeding = false;
    boolean wasOuttakeInRangeBefore = false;

    boolean idleVelocity = true;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        initializeMotors();


        waitForStart();


        while (opModeIsActive()) {
            if (isStopRequested()) return;



            stopAll();



            intake3Movement();
            double flipperPos = flipper.getPosition();
            flipperPos = flipper.getPosition();
            telemetry.addData("flipperPos",flipperPos);

            telemetry.update();
            intake2Movement();
            flip();
            controlIntake();
            moveOuttakeMotor();

            telemetry.update();
        }
    }







//NEW DATA POINTS 52 1050 58


    //DATA POINTS //61 1018 //79 1200//65 1080//60 1040//68 1110// 70 1150// 125 1350
    //NEW DATA ///
    private void flip(){
        if(gamepad1.rightStickButtonWasPressed()) {
            flipper.setPosition(0.5);

        }
        if(gamepad1.leftStickButtonWasPressed()){
            flipper.setPosition(0.8);
        }
    }


    private void controlIntake() {
        //Bring artifacts into the bot queue
        if (gamepad1.dpad_down) {
            intake.setPower(-0.4);
        }
        //Push artifacts out of the bot queue
        if (gamepad1.dpad_up) {
            intake.setPower(0.4);
        }
        //Stop intake with any other dpad or x.
        if (gamepad1.x) {
            intake.setPower(0);
        }
    }

    private void moveOuttakeMotor(){
        if(gamepad1.dpad_left){
            outtake.setPower(0.9);
        }
        if(gamepad1.dpad_right){
            outtake.setPower(0);
        }
    }

    private void stopAll(){
        if(gamepad1.aWasPressed()) {
            intake.setPower(0);
            intake2.setPower(0);
            intake3.setPower(0);
        }
    }



    private void intake2Movement(){
        if(gamepad1.leftBumperWasPressed()){
            intake2.setPower(-0.4);
        }
        if(gamepad1.rightBumperWasPressed()){
            intake2.setPower(0.4);
        }
    }
    private void intake3Movement(){
        if(gamepad1.bWasPressed()){
            intake3.setPower(-0.8);
        }
        if(gamepad1.yWasPressed()){
            intake3.setPower(0.8);
        }
    }





    private void initializeMotors() {

        intake = hardwareMap.crservo.get("intake");
        intake3 = hardwareMap.crservo.get("intake3");

;       flipper = hardwareMap.servo.get("flipper");
        intake2 = hardwareMap.crservo.get("intake2");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        //TODO: fix this to pull the correct object.
        //outtake = hardwareMap.get(DcMotorEx.class, "frontLeft");




    }



}
