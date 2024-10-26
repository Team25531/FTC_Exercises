package org.firstinspires.ftc.teamcode.neel;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Neel: Semicirlce", group = "Neel")
@Disabled//we need to add the DcMotors
public class semicircle extends LinearOpMode {
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
            fL.setPower(0.5);
            bL.setPower(0.5);
            fR.setPower(0.5);
            bR.setPower(0.5);

        }
        runtime.reset();

    }
    private void runSemicircle(DcMotor frontInnerMotor, DcMotor backInnerMotor, DcMotor frontOuterMotor, DcMotor backOuterMotor, double time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path 33", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            frontInnerMotor.setPower(0.1);
            backInnerMotor.setPower(0.1);
            frontOuterMotor.setPower(0.8);
            backOuterMotor.setPower(0.8);

        }
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
        runSemicircle(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, 2.3);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.2)) {
            frontLeftMotor.setPower(0.8);
            frontRightMotor.setPower(0.8);
            backRightMotor.setPower(0.8);


        }
        straightLine(frontLeftMotor, backLeftMotor, frontRightMotor, frontLeftMotor,2.4);



        runSemicircle(frontRightMotor, backRightMotor, frontLeftMotor, backLeftMotor, 3);
        while (opModeIsActive() && (runtime.seconds() < 3)) {
            frontLeftMotor.setPower(0.9);
            frontRightMotor.setPower(0.9);

            backLeftMotor.setPower(0.9);
        }
       straightLine(frontLeftMotor, backLeftMotor, frontRightMotor, frontLeftMotor,2.5);
    }
}
