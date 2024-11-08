package org.firstinspires.ftc.teamcode.Aaron;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public interface IImuProvider {

    public void initialize(HardwareMap hardwareMap) throws InterruptedException;
    public double getHeadingDegrees();
    public void resetYaw();
    public boolean isCalibrating();
}
