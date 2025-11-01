package org.firstinspires.ftc.teamcode.Aaron.y2024;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Aaron: DrivingOmni", group = "Aaron")
@Disabled
public class DrivingOmni extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("x", 3.7);
            packet.put("status", "alive");
            packet.fieldOverlay()
                    .setFill("blue")
                    .fillRect(-20, -20, 40, 40);

            FtcDashboard dashboard = FtcDashboard.getInstance();
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
