package org.firstinspires.ftc.teamcode.Aaron.y2024;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Aaron.Controller;

@TeleOp(name = "Aaron: MoveServo", group = "Aaron")
@Disabled
public class MoveServo extends LinearOpMode {
    Servo servo;
    Servo Second_servo;
    double position = 0;
    boolean buttonState;
    Controller gp;
  //  Pad pad;

    private CRServo finger;


        private Gamepad gamepad;
        private int xBut, yBut, aBut, bBut;

//        public Pad(Gamepad g) {
//            gamepad = g;
//        }

        public boolean XOnce() {
            return xBut == 1;
        }

        public boolean YOnce() {
            return yBut == 1;
        }

        public boolean AOnce() {
            return aBut == 1;
        }

        public boolean BOnce() {
            return bBut == 1;
        }

        public boolean X() {
            return 0 < xBut;
        }

        public boolean Y() {
            return 0 < yBut;
        }

        public boolean A() {
            return 0 < aBut;
        }

        public boolean B() {
            return 0 < bBut;
        }

        public void updateButtonState() {
            if (gamepad.x) {
                ++xBut;
            } else {
                xBut = 0;
            }
            if (gamepad.y) {
                ++yBut;
            } else {
                yBut = 0;
            }
            if (gamepad.a) {
                ++aBut;
            } else {
                aBut = 0;
            }
            if (gamepad.b) {
                ++bBut;
            } else {
                bBut = 0;
            }
        }




    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
//        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
//        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
//        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
//        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        //servo = hardwareMap.get(Servo.class, "servo");
        finger = hardwareMap.crservo.get("servo");
       // Second_servo = hardwareMap.get(Servo.class, "Second_servo");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
//        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
//        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        gp = new Controller(gamepad1);
        gamepad = gamepad1;
       // pad = new Pad(gamepad1);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            gp.update();
            updateButtonState();
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            if (gamepad1.left_bumper) {

                finger.setPower(1.0);
                telemetry.addData("left_bumper", buttonState);

                //tire moves inward to pull block in
            } else if (gamepad1.left_trigger > 0) {
                //tire moves outward to push block out
                finger.setPower(-1.0);
                telemetry.addData("left_trigger", buttonState);

            } else {
                finger.setPower(0);
                telemetry.addData("else", buttonState);

            }
//            frontLeftMotor.setPower(frontLeftPower);
//            backLeftMotor.setPower(backLeftPower);
//            frontRightMotor.setPower(frontRightPower);
//            backRightMotor.setPower(backRightPower);
//
//            if (gamepad1.x){
//                position = 0.18;
//
//            }
//            buttonState = gamepad1.x;
//
//            if (gamepad1.b) {
//                position = 0.1;
//                //  counter += 1;
//              //  Second_servo.setPosition(position);
//            }
//
//
//            if (gp.YOnce()) {
//                position += 0.10000;
//                position = Math.min(position, 1.0);
//            }
//            if (AOnce()) {
//                position -= 0.10000;
//                position = Math.max(position, 0);
//            }




//            double curPosition = Second_servo.getPosition();
//            if (!areEqualDouble(curPosition, position, 2)) {
//              //  Second_servo.setPosition(position);
//            }


//            telemetry.addData("Timer", "%5.2f", et.time());
//            telemetry.addData("Counter", counter);
//            telemetry.addData("a", aBut);
            telemetry.addData("buttonState", buttonState);
            telemetry.update();
        }

    }

    public static boolean areEqualDouble(double a, double b, int precision) {
        return Math.abs(a - b) <= Math.pow(10, -precision);
    }

}
