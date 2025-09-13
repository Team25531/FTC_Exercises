package org.firstinspires.ftc.teamcode.sandbox.Aaron;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.sandbox.neel.autoScore;

public class BackgroundElbowMove implements Runnable {

    private GlobalState state = null;

    public BackgroundElbowMove(GlobalState state) {
        this.state = state;
    }

    @Override
    public void run() {

        while (!state.isCancelled) {
           int thePosition = state.getElbowPosition();
  //          this.state.elbow.setPosition(thePosition);

            int tolerance = 4;
            int currentPosition = autoScore.elbow.getCurrentPosition();
            int elbowGoal = thePosition;
            if ((Math.abs(currentPosition - elbowGoal)) < tolerance) {
                return;
            }
            autoScore.elbow.setTargetPosition(elbowGoal);
            System.out.println("run elbow:" + Integer.toString(elbowGoal));

            autoScore.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autoScore.elbow.setPower(Math.abs(0.3));


            try {
                sleep(10);
            } catch (InterruptedException e) {
                System.out.println("StartElbowControl error state");
                throw new RuntimeException(e);
            }
        }
    }
}



