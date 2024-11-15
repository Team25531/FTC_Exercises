//package org.firstinspires.ftc.teamcode.Aaron;
//
//import org.firstinspires.ftc.teamcode.neel.autoScore;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//public class BackgroundPositionElbow implements Runnable {
//
//public volatile static int pos = -200;
//    @Override
//    public void run() {
//        int tolerance = 4;
//        int currentPosition = autoScore.elbow.getCurrentPosition();
//        int elbowGoal = pos;// autoScore.GetElbowGoal();
//        if ((Math.abs(currentPosition - elbowGoal)) < tolerance) {
//            return;
//        }
//        autoScore.elbow.setTargetPosition(elbowGoal);
//        autoScore.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        autoScore.elbow.setPower(Math.abs(0.8));
//
//    }
//}
