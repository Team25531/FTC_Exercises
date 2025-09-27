package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;


import java.util.List;

/*
 *
 * @see <a href="https://limelightvision.io/">Limelight</a>
 *
 * Notes on configuration:
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */

public class LimelightSensor {

    private Limelight3A limelight;

    public LimelightSensor (HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        //telemetry.setMsTransmissionInterval(11);
    }

    public List<LLResultTypes.ColorResult> detectPurple () {
        //detects the color purple
        //
        limelight.pipelineSwitch(0);
        limelight.start();
        LLResult result = limelight.getLatestResult();
        limelight.stop();
        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
        return colorResults;
    }

    public List<LLResultTypes.ColorResult> detectGreen () {
        //detects the color green
        limelight.pipelineSwitch(1);
        limelight.start();
        LLResult result = limelight.getLatestResult();
        limelight.stop();
        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
        return colorResults;
    }

    public List<LLResultTypes.FiducialResult> detectApriltag () {
        //detects april tags
        limelight.pipelineSwitch(9);
        limelight.start();
        LLResult result = limelight.getLatestResult();
        limelight.stop();
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        return fiducialResults;
    }

//    public static void main(String[] args) {
//        LimelightSensor limelightSensor = new LimelightSensor();
//
//         if LLResultTypes.ColorResult != null {
//            limelightSensor.detectPurple();
//        }
//
//    }
}
