package org.firstinspires.ftc.teamcode.Aaron;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//https://ftc-docs.firstinspires.org/en/latest/programming_resources/shared/pid_coefficients/pid-coefficients.html

//
//Random idea... To reduce the commands sent to the expansion hub and the possible load that braking (slowing) the outtake wheel causes, we could change the motor ZeroBehaivor from Brake to Float.
//
//        ```
//        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//  backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//```
//Then, when slowing down after a shot don't bother to decrease the power directly. Just set it to 0 (zero) and let it coast down to the IDLE_SPEED. The control loop can detect when it gets around IDLE+ a small amount, maybe, and turn power back on and start to maintain the IDLE speed.
////@Autonomous(name="Blue Alliance Auto", group="Pushbot", preselectTeleOp="BlueAllianceTeleOp")
//@Autonomous(name = "AutoDriveWithElbow", group = "auto", preselectTeleOp="FtcCompTeleop")
//@Disabled
//@Config
@TeleOp(name = "TestFeedback", group = "Aaron")
//@Disabled
public class TestFeedback extends LinearOpMode {
    public static int DashVariable = 15;

    private DcMotorEx backLeftMotor;
    private VoltageSensor myControlHubVoltageSensor;
    private VoltageSensor myExpansionHub2VoltageSensor;
    DigitalChannel digitalTouch;  // Digital channel Object
    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor;



    @Override
    public void runOpMode() throws InterruptedException {

        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        //myExpansionHub2VoltageSensor = hardwareMap.get(VoltageSensor.class, "Expansion Hub 2");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "button");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");

        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotorEx m = backLeftMotor;
       // Controller gp = new Controller(gamepad1);
        telemetry.addData("DigitalTouchSensorExample", "Press start to continue...");
        telemetry.update();
        waitForStart();

        FtcDashboard dashboard = FtcDashboard.getInstance();
       // Telemetry dashboardTelemetry = dashboard.getTelemetry();

        while (opModeIsActive()) {
            telemetry.addLine("running");
            if (gamepad1.aWasPressed()) {
                m.setPower(m.getPower() - 0.1);
            }

            if (gamepad1.xWasPressed()){
                m.setPower(m.getPower() + 0.1);
            }

            if (digitalTouch.getState() == false) {
                telemetry.addData("Button", "PRESSED");
            } else {
                telemetry.addData("Button", "NOT PRESSED");
            }
            //            https://github.com/WestsideRobotics/FTC-Power-Monitoring/wiki
            //            https://github.com/FIRST-Tech-Challenge/WikiSupport/blob/master/SampleOpModes/Datalogging/ConceptDatalogger.java
            //            https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Datalogging


            if (colorSensor instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
            }
//            telemetry.addData("Light Detected", ((OpticalDistanceSensor) colorSensor).getLightDetected());
       //     telemetry.addData("Light Detected", distanceSensor.getLightDetected());

//            telemetry.addData("Raw",    distanceSensor.getRawLightDetected());
//            telemetry.addData("Normal", distanceSensor.getLightDetected());

            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            telemetry.addData("Red", "%.3f", colors.red);
            telemetry.addData("Green", "%.3f", colors.green);
            telemetry.addData("Blue", "%.3f", colors.blue);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();

            packet.put("robotVoltage", myControlHubVoltageSensor.getVoltage());
            packet.put("getCurrentPosition", m.getCurrentPosition());
            packet.put("getPower", m.getPower());
            packet.put("getVelocity", m.getVelocity());
            packet.put("motorCurrent", m.getCurrent(CurrentUnit.AMPS));
            packet.put("DashVariable", DashVariable);

            dashboard.sendTelemetryPacket(packet);

        }
    }

//    private static class TimestampField// extends Datalogger.LoggableField
//    {
//        private long tRef;
//        private final DecimalFormat timeFmt = new DecimalFormat("000.000");
//
//        public TimestampField(String name)
//        {
//           // super(name);
//            tRef = System.currentTimeMillis();
//        }
//
//        public void resetRef()
//        {
//            tRef = System.currentTimeMillis();
//        }
//
//       // @Override
//        public void writeToBuffer(StringBuilder out)
//        {
//            long deltaMs = System.currentTimeMillis() - tRef;
//            float delta = deltaMs / 1000f;
//            out.append(timeFmt.format(delta));
//        }
//    }
}