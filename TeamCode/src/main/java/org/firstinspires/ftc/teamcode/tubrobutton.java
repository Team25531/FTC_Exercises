package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "General: tubrobutton", group = "General")
public class tubrobutton extends OpMode {
    @Override
    public void init() {

    }
    @Override
    public void loop(){
        if (gamepad1.a) {
            telemetry.addData("Foward Speed", gamepad1.left_stick_x );

        }
        else {
            telemetry.addData("Foward Speed", gamepad1.left_stick_x * 0.5);
        }

    }
}
