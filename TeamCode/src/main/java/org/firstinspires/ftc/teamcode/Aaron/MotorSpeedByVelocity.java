package org.firstinspires.ftc.teamcode.Aaron;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@SuppressLint("DefaultLocale")
@TeleOp(name = "MotorSpeedByVelocity", group = "Aaron")
//@Disabled
public class MotorSpeedByVelocity extends LinearOpMode {


    @Override
    public void runOpMode() {
        Controller c = new Controller(gamepad1);
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        frontRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElapsedTime runtime = new ElapsedTime();

        telemetry.addData(">", "OpMode to control motor speed by adjusting the velocity.");
        telemetry.addData(">", "Press START to start OpMode");
        telemetry.update();

        int targetA = 400;
        int targetY = 2000;
        int ticksPerRev = 1440;
        double targetVelocity = 0;
        int timeToSpeed = 0;
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                c.update();

                double curVelocity = frontRightMotor.getVelocity();
                telemetry.addData("Curr Ticks/sec:", curVelocity);

                double RPMs = curVelocity / ticksPerRev;
                telemetry.addLine(String.format("Curr RPM: %6.2f", RPMs));

                if (c.BOnce()) {
                    targetVelocity = 0;
                }
                if (c.AOnce()) {
                    targetVelocity = targetA;
                    runtime.reset();
                    timeToSpeed = 0;
                }
                if (c.YOnce()) {
                    targetVelocity = targetY;
                    runtime.reset();
                    timeToSpeed = 0;
                }

                frontRightMotor.setVelocity(targetVelocity);

                telemetry.addData("targetVelocity", targetVelocity);

                double range = targetVelocity * 0.05; //xx% of the target
                double minRange = targetVelocity - range;
                double maxRange = targetVelocity + range;

                if (targetVelocity > 0 && curVelocity >= minRange && curVelocity <= maxRange) {
                    if (timeToSpeed == 0) {
                        timeToSpeed = (int) runtime.milliseconds();
                    }
                    telemetry.addData("At Target", "SHOOT NOW!!");
                    telemetry.addLine(String.format("Time to Target: %dms", timeToSpeed));
                } else {
                    telemetry.addData("At Target", "wait");
                }
                telemetry.update();
            }
        }
    }
}