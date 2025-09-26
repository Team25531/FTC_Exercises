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


    @Override

    public void runOpMode() throws InterruptedException {
        waitForStart();

        DcMotor test = hardwareMap.dcMotor.get("test");


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

            if (gamepad1.dpad_up) {

               test.setPower(10);
                //debug
                telemetry.addData("dpadUP", 0);
                telemetry.update();



                //makes arm go up
            }



        }




    }

}




