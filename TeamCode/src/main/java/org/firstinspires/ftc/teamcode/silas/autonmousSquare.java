package org.firstinspires.ftc.teamcode.silas;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class autonmousSquare {

    public class StraightLine extends LinearOpMode {
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        private DcMotor backLeftMotor = hardwareMap.dcMotor.get("rearLeft");
        private DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        private DcMotor backRightMotor = hardwareMap.dcMotor.get("rearRight");

        private void straightLine() {
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.0)) {
                telemetry.addData("Path 11", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
                frontLeftMotor.setPower(0.5);
                backLeftMotor.setPower(0.5);
                frontRightMotor.setPower(0.5);
                backRightMotor.setPower(0.5);
            }
            runtime.reset();
        }
        @Override
        public void runOpMode() throws InterruptedException {

            telemetry.addData("Path 00", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();

            frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            waitForStart();

            straightLine();

            frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
            backRightMotor.setDirection(DcMotor.Direction.FORWARD);

            straightLine();
        }
    }
}
