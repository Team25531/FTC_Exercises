package org.firstinspires.ftc.teamcode.neel;
/*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.json.JSONObject;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.Scanner;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/*
 // Pipeline 0 = Green
  Pipeline 1 = purple
   Pipeline 9 = april tag detection.
 */
@TeleOp(name = "limelightTest", group = "Sensor")
@Disabled

public class limelightTest extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException
    {
         limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(2000);



        limelight.pipelineSwitch(0);

        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            String detectedColor = "UNKNOWN";

            LLResult result = limelight.getLatestResult();
            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
            for (LLResultTypes.ColorResult cr : colorResults) {
                telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
            }
            if (gamepad1.dpad_down) {
                limelight.pipelineSwitch(1);

            }
            else if (gamepad1.dpad_up){
                limelight.pipelineSwitch(0);
            }


            telemetry.update();

//            try {
//
//                URL url = new URL(limelight + "/json");
//                HttpURLConnection conn = (HttpURLConnection) url.openConnection();
//                conn.setRequestMethod("GET");
//
//                Scanner scanner = new Scanner(conn.getInputStream());
//                StringBuilder response = new StringBuilder();
//                while (scanner.hasNext()) {
//                    response.append(scanner.nextLine());
//                }
//                scanner.close();
//
//                JSONObject json = new JSONObject(response.toString());
//
//                // "pID" tells us which pipeline is active
//                int pipeline = json.getInt("pID");
//
//                if (pipeline == 0) {
//                    detectedColor = "GREEN";
//                } else if (pipeline == 1) {
//                    detectedColor = "PURPLE";
//                }
//
//            } catch (Exception e) {
//                detectedColor = "ERROR: " + e.getMessage();
//            }
//
//            telemetry.addData("Detected Color", detectedColor);
//            telemetry.update();

//            sleep(100); // small delay
        }



        }
        ;
    }

