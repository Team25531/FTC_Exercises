package org.firstinspires.ftc.teamcode.sandbox.mohit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Mohit: StraightLine", group = "Mohit")
@Disabled//we need to add the DcMotors
public class MohitStraightLine extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("rearLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rearRight");
        telemetry.addData("Path 00", "Leg 1: %4.1f S Elapsed", runtime.seconds());
        telemetry.update();

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
       // resetRuntime();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 5.0)) {
            telemetry.addData("Path 11", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            frontLeftMotor.setPower(0.8);
            backLeftMotor.setPower(0.8);
            frontRightMotor.setPower(0.8);
            backRightMotor.setPower(0.8);
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 5.0)) {
            telemetry.addData("Path 22", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            frontLeftMotor.setPower(0.8);
            backLeftMotor.setPower(0.8);
            frontRightMotor.setPower(0.8);
            backRightMotor.setPower(0.8);


        }
    }
}



