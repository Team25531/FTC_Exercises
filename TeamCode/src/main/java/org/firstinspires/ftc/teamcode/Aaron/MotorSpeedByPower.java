package org.firstinspires.ftc.teamcode.Aaron;

import android.annotation.SuppressLint;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@SuppressLint("DefaultLocale")
@TeleOp(name = "MotorSpeedByPower", group = "Aaron")
@Disabled
public class MotorSpeedByPower extends LinearOpMode {


    @Override
    public void runOpMode() {
        Controller c = new Controller(gamepad1);
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        frontRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElapsedTime runtime = new ElapsedTime();

        telemetry.addData(">", "OpMode to control motor speed by adjusting the power.");
        telemetry.addData(">", "Press START to start OpMode");
        telemetry.update();

        int targetA = 400;
        int targetY = 2000;
        int ticksPerRev = 1440;
        double targetTicks = 0;
        double power = 0;
        int timeToSpeed = 0;

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                c.update();

                double curTicks = frontRightMotor.getVelocity();
                telemetry.addData("Curr Ticks/sec:", curTicks);

                double RPMs = curTicks / ticksPerRev;
                telemetry.addLine(String.format("Curr RPM: %6.2f", RPMs));

                if (c.B()) {
                    targetTicks = 0;
                }
                if (c.A()) {
                    targetTicks = targetA;
                    runtime.reset();
                    timeToSpeed = 0;
                }
                if (c.Y()) {
                    targetTicks = targetY;
                    runtime.reset();
                    timeToSpeed = 0;
                }
                double range = targetTicks * 0.05; //xx% of the target
                double minRange = targetTicks - range;
                double maxRange = targetTicks + range;

                power = getPowerByIncrement(curTicks, targetTicks, power, minRange, maxRange);
                //power = getPowerByProportion(curTicks, targetTicks, power, minRange, maxRange);
                frontRightMotor.setPower(power);

                telemetry.addData("targetVelocity", targetTicks);
                telemetry.addData("power applied", power);

                if (targetTicks > 0 && curTicks >= minRange && curTicks <= maxRange) {
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

    private double getPowerByIncrement(double curTicks, double targetTicks, double power, double minRange, double maxRange) {
        double powerFactor = 0.001;
        //double powerFactor = 0.05;
        telemetry.addData("power factor", powerFactor);

        int which = 0;
        if (curTicks < minRange) {
            power += powerFactor;
            which = 1;
        } else if (curTicks > maxRange) {
            power -= powerFactor;
            which = 2;
        }
        if (targetTicks == 0) {
            power = 0;
            which = 3;
        }
        power = Range.clip(power, -1, 1);
        Log.d("PowerFactor", String.valueOf(which) + " - " + String.valueOf(curTicks) + " - " + String.valueOf(targetTicks) + " - " + String.valueOf(power) + " - " + String.valueOf(powerFactor));

        return power;
    }


//    private double getPowerByProportion(double curTicks, double targetTicks, double power, double minRange, double maxRange) {
//        //double powerFactor = 0.001;
//        double powerFactor = 0;
//
//        int which = 0;
//        if (curTicks < minRange) {
//            powerFactor = 1.0 - (curTicks / targetTicks);
//            power += powerFactor;
//            which = 1;
//        } else if (curTicks > maxRange) {
//            powerFactor = Math.abs(curTicks / targetTicks);
//            power -= powerFactor;
//            which = 2;
//        } else {
//
//        }
//        if (targetTicks == 0) {
//            power = 0;
//            which = 3;
//        }
//        power = Range.clip(power, -1, 1);
//        telemetry.addData("power factor", powerFactor);
//        Log.d("PowerFactor", String.valueOf(which) + " - " + String.valueOf(curTicks) + " - " + String.valueOf(targetTicks) + " - " + String.valueOf(power) + " - " + String.valueOf(powerFactor));
//        return power;
//    }

}