package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name = "SampleClassWithDriving", group = "Common")
//@Disabled
public class SampleClassWithDriving extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive mec = new MecanumDrive(this); //leave this line in place
        mec.UseYForSpeedReduction = true;//optional, Y Button reduces speed by 2.5X
        mec.ShowTelemetry = true; //optional, add each motor power to telemetry

        //Your code here

        waitForStart();

        while (opModeIsActive()) {
            mec.Drive(); //leave this line in place

            //Your code here

            telemetry.update();
        }
    }
}