package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDrive {

    private final LinearOpMode opMode;
    private final Telemetry telemetry;
    private final Gamepad gp;

    public boolean UseYForSpeedReduction;
    public boolean ShowTelemetry;

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    public MecanumDrive(LinearOpMode op) {
        opMode = op;
        gp = opMode.gamepad1;
        telemetry = opMode.telemetry;
        Initialize(opMode.hardwareMap);
    }

    public void Initialize(HardwareMap hm) {
        frontLeftMotor = hm.dcMotor.get("frontLeft");
        backLeftMotor = hm.dcMotor.get("backLeft");
        frontRightMotor = hm.dcMotor.get("frontRight");
        backRightMotor = hm.dcMotor.get("backRight");

        assert frontLeftMotor != null;
        assert backLeftMotor != null;
        assert frontRightMotor != null;
        assert backRightMotor != null;

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void Drive() {
        if (opMode.opModeIsActive()) {
            double y = -gp.left_stick_y; // Remember, Y stick value is reversed
            double x = gp.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gp.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            // speedModifier : Larger the value, the lower the top speed is. Usefully for setting a speed cap.
            // buttonModifier : if the Y button is pressed then lower the seed further for better control
            double speedModifier = Math.abs(1.5); //
            double buttonModifier = (UseYForSpeedReduction && gp.y) ? 2.5 : 1.0;
            double denominator = (Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1))
                    * speedModifier * buttonModifier;

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if (ShowTelemetry) {
                telemetry.addData("frontLeftPower", frontLeftPower);
                telemetry.addData("backLeftPower", backLeftPower);
                telemetry.addData("frontRightPower", frontRightPower);
                telemetry.addData("backRightPower", backRightPower);
            }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        } else {
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
        }
    }
}
