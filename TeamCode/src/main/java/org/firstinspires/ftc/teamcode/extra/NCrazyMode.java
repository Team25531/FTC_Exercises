package org.firstinspires.ftc.teamcode.extra;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@TeleOp(name = "General: NCrazyMode", group = "General")
@Disabled
public class NCrazyMode extends OpMode {
    @Override
    public void init() {

    }
    @Override
    public void loop(){
        if (gamepad1.a) {
            telemetry.addData("Right Stick", gamepad1.left_stick_x );
            telemetry.addData("left stick", gamepad1.right_stick_x);

        }
        else {
            telemetry.addData("Right Stick", gamepad1.right_stick_x);
            telemetry.addData("left stick", gamepad1.left_stick_x);
        }

    }
}



