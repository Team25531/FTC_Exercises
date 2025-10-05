package org.firstinspires.ftc.teamcode.Aaron.y2024;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;


public class ImuNavX2Micro implements IImuProvider {
    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;
    private AHRS navx_device;

    @Override
    public void initialize(HardwareMap hardwareMap) throws InterruptedException {
      //  navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
      //  gyro = (IntegratingGyroscope)navxMicro;
      //  navxMicro.initialize();
      //  navxMicro.resetDeviceConfigurationForOpMode();

        navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData);
        while(navx_device.isCalibrating()){
            Thread.sleep(50);
        }
    }

    @Override
    public double getHeadingDegrees() {
//        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        return angles.firstAngle;
        return navx_device.getYaw();
    }

    @Override
    public void resetYaw() {
        navx_device.zeroYaw();
    }

    @Override
    public boolean isCalibrating() {
        return navx_device.isCalibrating();
    }
}
