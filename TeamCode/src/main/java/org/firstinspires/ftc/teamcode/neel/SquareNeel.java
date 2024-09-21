package org.firstinspires.ftc.teamcode.neel;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Neel: SquareNeel", group = "Neel")
//we need to add the DcMotors
public class SquareNeel extends LinearOpMode {
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
            bL.setPower(0.5 );
            fR.setPower(0.5);
            bR.setPower(0.5);

        }
        runtime.reset();

    }




    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        // semicircle


        straightLine(frontLeftMotor, backLeftMotor, frontRightMotor, frontLeftMotor,2.5);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            frontLeftMotor.setPower(-5);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(5);
            backLeftMotor.setPower(0);


        }
        straightLine(frontLeftMotor, backLeftMotor, frontRightMotor, frontLeftMotor,2.5);



       runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            frontLeftMotor.setPower(-5);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(5);
            backLeftMotor.setPower(-5);
        }
        straightLine(frontLeftMotor, backLeftMotor, frontRightMotor, frontLeftMotor,2.5);
    }
}
