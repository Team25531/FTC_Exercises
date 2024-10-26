package org.firstinspires.ftc.teamcode.Aaron;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "motorHolding", group = "Aaron")
@Disabled
public class motorHolding extends LinearOpMode {
    int HoldPosition;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor elbow = hardwareMap.dcMotor.get("elbow");
        elbow.setTargetPosition(0);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            telemetry.addData("Currently goal", HoldPosition);
            telemetry.addData("Currently   at", elbow.getCurrentPosition());

            if (gamepad1.dpad_up) {
                elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elbow.setPower(-0.8);
                HoldPosition = elbow.getCurrentPosition();
            } else if (gamepad1.dpad_down) {
                elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elbow.setPower(0.8);
                HoldPosition = elbow.getCurrentPosition();
            } else {
                elbow.setPower(0);
                HoldArmStill(HoldPosition, elbow);
            }
            telemetry.update();
        }
    }

    public void HoldArmStill(int posToHold, DcMotor motor) {
        int tolerance = 4;
        int currentPosition = motor.getCurrentPosition();
        if ((Math.abs(currentPosition - posToHold)) < tolerance) {
            return;
        }
        motor.setTargetPosition(posToHold);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(Math.abs(0.8));
    }
}
