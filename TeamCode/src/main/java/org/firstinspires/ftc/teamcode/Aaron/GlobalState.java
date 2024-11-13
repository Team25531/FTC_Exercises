package org.firstinspires.ftc.teamcode.Aaron;

import com.qualcomm.robotcore.hardware.DcMotor;

public class GlobalState {
    public DcMotor elbow;
    volatile int elbowPosition = 0;
    public volatile boolean isCancelled = false;

    public void setElbowPosition(int position) {
        elbowPosition = position;
    }
    public int getElbowPosition(){
        return elbowPosition;
    }

}