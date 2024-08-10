package org.firstinspires.ftc.teamcode.neel;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Neel: Semicirlce", group = "Neel")
//we need to add the DcMotors
public class semicircle extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
    private DcMotor backLeftMotor = hardwareMap.dcMotor.get("rearLeft");
    private DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
    private DcMotor backRightMotor = hardwareMap.dcMotor.get("rearRight");

    private void straightLine() {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            telemetry.addData("Path 11", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            frontLeftMotor.setPower(0.8);
            backLeftMotor.setPower(0.8);
            frontRightMotor.setPower(0.8);
            backRightMotor.setPower(0.8);

        }
        runtime.reset();

    }
    @Override
    public void runOpMode() throws InterruptedException {

        // semicircle
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            telemetry.addData("Path 33", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            frontLeftMotor.setPower(0.2);
            backLeftMotor.setPower(0.2);
            frontRightMotor.setPower(0.8);
            backRightMotor.setPower(0.8);

        }
    }
}
