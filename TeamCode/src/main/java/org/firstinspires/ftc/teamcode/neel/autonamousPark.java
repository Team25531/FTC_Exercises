package org.firstinspires.ftc.teamcode.neel;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Neel: Semicirlce", group = "Neel")
//we need to add the DcMotors
public class autonamousPark extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor;// = hardwareMap.dcMotor.get("frontLeft");
    private DcMotor backLeftMotor;// = hardwareMap.dcMotor.get("backLeft");
    private DcMotor frontRightMotor;// = hardwareMap.dcMotor.get("frontRight");
    private DcMotor backRightMotor;// = hardwareMap.dcMotor.get("backRight");

    private void straightLine(DcMotor fL,DcMotor bL, DcMotor fR, DcMotor bR, double time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path 11", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            fL.setPower(0.3);
            bL.setPower(0.3);
            fR.setPower(0.3);
            bR.setPower(0.3);

        }
        runtime.reset();

    }



    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();
        // semicircle
        resetRuntime();
        straightLine(frontLeftMotor, backLeftMotor, frontRightMotor, frontLeftMotor,0.14);
        resetRuntime();
        getRuntime();
        resetRuntime();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            frontLeftMotor.setPower(0.9);
            frontRightMotor.setPower(0.9);

            backLeftMotor.setPower(0.9);
        }
        straightLine(frontLeftMotor, backLeftMotor, frontRightMotor, frontLeftMotor,3);
    }
}

