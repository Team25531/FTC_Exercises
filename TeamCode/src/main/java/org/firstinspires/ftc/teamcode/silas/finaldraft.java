package org.firstinspires.ftc.teamcode.silas;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name = "final draft", group = "TeleOp")
@Disabled
public class finaldraft extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor topfrontMotor;

    @Override
    public void runOpMode() {

        frontLeftMotor = hardwareMap.dcMotor.get( "frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get( "rearLeft");
        frontRightMotor = hardwareMap.dcMotor.get( "frontRight");
        backRightMotor = hardwareMap.dcMotor.get( "frontLeft");
        topfrontMotor = hardwareMap.dcMotor.get("topfront");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        topfrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        if (gamepad1.dpad_down){
            topfrontMotor.setPower(0.1);
            }

        }
    }