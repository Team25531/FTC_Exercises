package org.firstinspires.ftc.teamcode.Aaron;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "TestFeedback", group = "Aaron")
@Disabled
public class TestFeedback extends LinearOpMode {

    private DcMotorEx backLeftMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotorEx m = backLeftMotor;
        Controller gp = new Controller(gamepad1);
        waitForStart();

        while (opModeIsActive()) {
            gp.update();
            double power = Range.clip(100, -1, 1);
            telemetry.addLine("running");
            if (gp.A()) {
                // m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addData("a pressed", power);
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m.setPower(power);
            }

            if (gp.BOnce()) {
                telemetry.addData("b pressed", 0);
                int curPos = m.getCurrentPosition();
                int rpt = 1400;
                int rem = curPos % rpt;
                int target = rem;
                m.setTargetPosition(rem);
                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                m.setPower(0.3);
                while (m.isBusy() && opModeIsActive()) {
                    sleep(1);
                }
                m.setPower(0);
            }

            telemetry.addLine(String.valueOf(m.getCurrentPosition()));
            telemetry.update();
        }
    }
}