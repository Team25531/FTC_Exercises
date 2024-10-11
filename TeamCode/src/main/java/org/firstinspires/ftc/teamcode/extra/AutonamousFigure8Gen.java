package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "General: Autonomous8", group = "General")
//we need to add the DcMotors
public class AutonamousFigure8Gen extends LinearOpMode {
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

        waitForStart();
       // resetRuntime();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 5.0)) {
            telemetry.addData("Path 11", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            frontLeftMotor.setPower(0.8);
            backLeftMotor.setPower(0.8);
            frontRightMotor.setPower(0.5);
            backRightMotor.setPower(0.5);
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 15.0)) {
            telemetry.addData("Path 22", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            frontLeftMotor.setPower(0.5);
            backLeftMotor.setPower(0.5);
            frontRightMotor.setPower(0.8);
            backRightMotor.setPower(0.8);


        }
    }
}



