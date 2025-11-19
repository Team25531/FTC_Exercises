package org.firstinspires.ftc.teamcode.neel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Light Testing")
public class LightTesting extends LinearOpMode {

    private Servo light;

    @Override
    public void runOpMode() {

        // Map the RGB indicator as a servo
        light = hardwareMap.get(Servo.class, "light");

        waitForStart();


        while (opModeIsActive()) {

            if (gamepad1.a) {
                // A pressed → Red light
                light.setPosition(0.05);   // Red (0.0–0.1 range)
            } else {
                // A released → Off
                light.setPosition(0.95);   // Off (0.9–1.0 range)
            }

            telemetry.addData("A Pressed?", gamepad1.a);
            telemetry.update();
        }
    }
}
