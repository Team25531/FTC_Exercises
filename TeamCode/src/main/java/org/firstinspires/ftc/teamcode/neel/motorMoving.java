package org.firstinspires.ftc.teamcode.neel;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Neel: TEST ", group = "Neel")

public class motorMoving extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor test;
    double motorTestDirection = -1;
    private CRServo intake;
    boolean motorON = false;

    boolean motorDirection = false;


    @Override

    public void runOpMode() throws InterruptedException {
        waitForStart();

        DcMotor test = hardwareMap.dcMotor.get("test");
        intake = hardwareMap.crservo.get("intake");


        //reseting variable called runtime
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]



            runtime.reset();


            if (gamepad1.y) {

               test.setPower(-1 * motorTestDirection);

                //debug
                telemetry.addData("y", 0);
                telemetry.update();

                motorON = true;
                telemetry.addData("Motor on",0);



                //makes arm go up
            }
            if (gamepad1.x){
                test.setPower(-0.8 * motorTestDirection);

            }
            if (gamepad1.a){
                test.setPower(-0.6 * motorTestDirection);

            }
            if (gamepad1.b){
                test.setPower(-0.4 * motorTestDirection);

            }

            if (gamepad1.dpad_down){
                intake.setPower(0.8);
            }
            if(gamepad1.dpad_up){
                intake.setPower(-0.8);
            }

        }


    }

}




