package org.firstinspires.ftc.teamcode.Santiago;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Tank Drive", group = "Santiago")
public class autoTank extends LinearOpMode {
    private DcMotor frontLeftMotor;// = hardwareMap.dcMotor.get("frontLeft");
    private DcMotor backLeftMotor;// = hardwareMap.dcMotor.get("backLeft");
    private DcMotor frontRightMotor;// = hardwareMap.dcMotor.get("frontRight");
    private DcMotor backRightMotor;// = hardwareMap.dcMotor.get("backRight");
    private CRServo fanTest;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        CRServo fanTest = hardwareMap.crservo.get("fanTest");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        while (opModeIsActive()) {
            double forwardBackward = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double leftRight = gamepad1.right_stick_x;
            double leftPower = forwardBackward + (leftRight);
            double rightPower = forwardBackward - (leftRight);
            double fanPower = 0;
            if (gamepad1.a) {
                fanPower = 0.1;
            }
            else if (gamepad1.x) {
                fanPower = 0.2;
            }
            else if (gamepad1.b) {
                fanPower = 0.3;
            }
            else if (gamepad1.y) {
                fanPower = 0.4;
            }
            backLeftMotor.setPower(leftPower);
            frontLeftMotor.setPower(leftPower);
            backRightMotor.setPower(rightPower);
            frontRightMotor.setPower(rightPower);

            fanTest.setPower(fanPower);
            telemetry.addData("Forward and Backward",forwardBackward);
            telemetry.addData("Left and Right",leftRight);

            if (gamepad1.dpad_right) {
                double frontCurrentPower = frontLeftMotor.getPower();
                double backCurrentPower = backLeftMotor.getPower();
                frontLeftMotor.setPower(frontCurrentPower + 0.2);
                backLeftMotor.setPower(backCurrentPower + 0.2);
            }
            if (gamepad1.dpad_left) {
                double frontCurrentPower = frontRightMotor.getPower();
                double backCurrentPower = backRightMotor.getPower();
                frontRightMotor.setPower(frontCurrentPower + 0.2);
                backRightMotor.setPower(backCurrentPower + 0.2);
            }

            telemetry.update();
        }
    }
}