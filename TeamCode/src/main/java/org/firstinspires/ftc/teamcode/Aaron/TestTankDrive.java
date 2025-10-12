package org.firstinspires.ftc.teamcode.Aaron;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TestTankDrive", group = "Aaron")
public class TestTankDrive extends LinearOpMode {
    private DcMotor frontLeftMotor;// = hardwareMap.dcMotor.get("frontLeft");
    private DcMotor backLeftMotor;// = hardwareMap.dcMotor.get("backLeft");
    private DcMotor frontRightMotor;// = hardwareMap.dcMotor.get("frontRight");
    private DcMotor backRightMotor;// = hardwareMap.dcMotor.get("backRight");


    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");


        //setting direction for motors


        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();

        while (opModeIsActive()) {

            double forwardPower = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double leftRight = gamepad1.right_stick_x;

            telemetry.addData("forwardPower ", forwardPower);
            telemetry.addData("leftRight ", leftRight);

            double leftPower = forwardPower;
            double rightPower = forwardPower;

                leftPower = forwardPower + (leftRight / 2);
                rightPower = forwardPower - (leftRight / 2);

            telemetry.addData("leftPower ", leftPower);
            telemetry.addData("rightPower ", rightPower);

            frontLeftMotor.setPower(leftPower);
            backLeftMotor.setPower(leftPower);
            frontRightMotor.setPower(rightPower);
            backRightMotor.setPower(rightPower);

            telemetry.update();
        }
    }
}
