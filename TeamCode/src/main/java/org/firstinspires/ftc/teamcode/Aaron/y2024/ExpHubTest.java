package org.firstinspires.ftc.teamcode.Aaron.y2024;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Aaron.Controller;


@TeleOp(name = "Aaron: ExpHubTest", group = "Aaron")
@Disabled
public class ExpHubTest extends LinearOpMode {
    Servo servo;
   // Servo Second_servo;
    double position = 0;
    Controller gp;
  //  Pad pad;



    enum State{Start, Run, Walk, Jump, Stop}
    State state;

    public void StateMachine()
    {
        state = State.Start;

        LOOP:
        while(true)
        {
            switch (state){
                case Start:
                    telemetry.addLine("state: start");
                    telemetry.addLine("slf");
                    break;
                case Jump:
                    break;

                default:
                    break LOOP;



            }
        }
    }


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
        servo = hardwareMap.get(Servo.class, "servo");

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



            if (gamepad1.x){
                position = 0.08;
                servo.setPosition(position);
            }


            if (gp.YOnce()) {
                position += 0.10000;
                position = Math.min(position, 1.0);
            }
            if (AOnce()) {
                position -= 0.10000;
                position = Math.max(position, 0);
            }




            double curPosition = servo.getPosition();
            if (!areEqualDouble(curPosition, position, 2)) {
                servo.setPosition(position);
            }

//            telemetry.addData("Timer", "%5.2f", et.time());
//            telemetry.addData("Counter", counter);
//            telemetry.addData("a", aBut);
//            telemetry.addData("buttonDown", buttonDown);
            telemetry.update();
        }

    }

    public static boolean areEqualDouble(double a, double b, int precision) {
        return Math.abs(a - b) <= Math.pow(10, -precision);
    }

}
