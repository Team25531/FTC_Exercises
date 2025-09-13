//package org.firstinspires.ftc.teamcode.neel;
//
//
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@Autonomous(name = "Autodrive", group = "FTC Comp")
////we need to add the DcMotors
//public class autodrive extends LinearOpMode {
//    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor frontLeftMotor;// = hardwareMap.dcMotor.get("frontLeft");
//    private DcMotor backLeftMotor;// = hardwareMap.dcMotor.get("backLeft");
//    private DcMotor frontRightMotor;// = hardwareMap.dcMotor.get("frontRight");
//    private DcMotor backRightMotor;// = hardwareMap.dcMotor.get("backRight");
//
//    private void straightLine(DcMotor fL, DcMotor bL, DcMotor fR, DcMotor bR, double time) {
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < time)) {
//            telemetry.addData("Path 11", "Leg 1: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//            fL.setPower(0.5);
//            fR.setPower(0.5);
//            bL.setPower(0.5);
//            bR.setPower(0.5);
//
//        }
//        runtime.reset();
//
//    }
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
//        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
//        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
//        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
//
//        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        frontRightMotor.setDirection((DcMotor.Direction.FORWARD);
//        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
//        waitForStart();
//        // semicircle
//
//        runtime.reset();
//        straightLine(frontLeftMotor, backLeftMotor, frontRightMotor, frontLeftMotor, 0.2);
//        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
//        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
//        while (opModeIsActive() && (runtime.seconds() < 4 )) {
//            frontLeftMotor.setPower(0.3);
//            backLeftMotor.setPower(0.3);
//            backRightMotor.setPower(0.3);
//            frontRightMotor.setPower(0.3);
//
//
//        }
//
//
//    }
//}