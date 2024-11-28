//package org.firstinspires.ftc.teamcode.Aaron;
//
//
//import static java.lang.Thread.sleep;
//
//import com.qualcomm.robotcore.hardware.Servo;
//
//
//public class BackgroundServoMove implements Runnable {
//
//    GlobalState instance = null;
//
//    public BackgroundServoMove(GlobalState instance) {
//        this.instance = instance;
//    }
//
//    @Override
//    public void run() {
//
//        if (instance.leftServoPosition != 0.0) {
//            this.instance.leftServo.setPosition(instance.leftServoPosition);
//
//        } else {
//            int i = 0;
//            double left = 0.0;
//
//
//            while (i < 100) {
//                i++;
//                left += .01;
//                if (left > 1) left = 0;
//
//                this.instance.leftServo.setPosition(left);
//                try {
//                    sleep(100);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
//
//            }
//        }
//    }
////    public final static double pos = 0.2;
////     Servo servo;
////
////    @Override
////    public void run() {
////
////
////        int i = 0;
////        double left = 0.0;
////
////        while (i < 100) {
////            i++;
////            left += .01;
////            if (left > 1) left = 0;
////
////            servo.setPosition(left);
////
////        }
//
//}
//
//
//
