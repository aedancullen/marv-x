package org.firstinspires.ftc.teamcode;

import com.evolutionftc.autopilot.AutopilotSystem;
import com.evolutionftc.autopilot.AutopilotTracker;
import com.evolutionftc.autopilot.AutopilotTrackerQP37i;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Marv Mk10 AP Track Test")
public class MarvXAPTrackTest extends OpMode {

    MarvXCommon marv;

    AutopilotTracker tracker;
    AutopilotSystem ap;

    public void init() {
        marv = new MarvXCommon(hardwareMap);
        marv.setEncoderBehavior(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        marv.setEncoderBehavior(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tracker = new AutopilotTrackerQP37i(marv.getQuadPacerMotorX(), marv.getQuadPacerMotorY(), new double[3], MarvNavConstants.QUADPACER_TPU, marv.imu, 1);
        ap = new AutopilotSystem(tracker, telemetry, hardwareMap.appContext, true);
    }

    public void loop() {
        ap.systemTick();
    }

}
