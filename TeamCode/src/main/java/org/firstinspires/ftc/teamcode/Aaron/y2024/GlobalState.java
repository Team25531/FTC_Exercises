package org.firstinspires.ftc.teamcode.Aaron.y2024;

import com.qualcomm.robotcore.hardware.DcMotor;

public class GlobalState {
    public DcMotor elbow;
    volatile int elbowPosition = 0;
    public volatile boolean isCancelled = false;

    public void setElbowPosition(int position) {
        System.out.println("setElbowPosition original:" + Integer.toString(elbowPosition));
        System.out.println("setElbowPosition new:" + Integer.toString(position));

        elbowPosition = position;
    }
    public int getElbowPosition(){
        return elbowPosition;
    }

}