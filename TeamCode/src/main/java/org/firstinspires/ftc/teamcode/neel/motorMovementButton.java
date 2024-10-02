package org.firstinspires.ftc.teamcode.neel;

//imports are used so you can use a code defined from a different system
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Neel: Button movement", group = "Neel")
//we need to add the DcMotors
//this code makes it so it s able to be shared
public class motorMovementButton extends LinearOpMode {

    //This code sets up a timer and declares motors for the robot's movement (front left, back left, front right, back right, elbow) and a servo for a claw or finger.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor;// = hardwareMap.dcMotor.get("frontLeft");
    private DcMotor backLeftMotor;// = hardwareMap.dcMotor.get("backLeft");
    private DcMotor frontRightMotor;// = hardwareMap.dcMotor.get("frontRight");
    private DcMotor backRightMotor;// = hardwareMap.dcMotor.get("backRight");
    private DcMotor elbow;// = hardwareMap.dcMotor.get("backRight");
    private Servo finger;

    int position = 0;


    // this code makes it so it dosen't have to follow all the rules fromt he omports
    @Override
    public void runOpMode() throws InterruptedException {
        //this code is used for defining motor
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        DcMotor elbow = hardwareMap.dcMotor.get("elbow");
        finger = hardwareMap.get(Servo.class, "finger");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        elbow.setDirection(DcMotor.Direction.REVERSE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                elbow.setPower(0.8);
            }
            else if (gamepad1.dpad_down) {
                elbow.setPower(-0.8);
                //going down
            }
            else {
                elbow.setPower(0);

            }
            while (gamepad1.dpad_left){
                position = 10;
                finger.setPosition(position);
            }
            while (gamepad1.dpad_right){
                position = -10;
                finger.setPosition(position);
            }

        }
    }

    }



