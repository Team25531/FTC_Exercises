package org.firstinspires.ftc.teamcode.Aaron.y2024;
import com.qualcomm.robotcore.hardware.HardwareMap;

public interface IImuProvider {

    public void initialize(HardwareMap hardwareMap) throws InterruptedException;
    public double getHeadingDegrees();
    public void resetYaw();
    public boolean isCalibrating();
}
