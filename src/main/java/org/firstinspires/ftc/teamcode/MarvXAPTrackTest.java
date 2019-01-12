package org.firstinspires.ftc.teamcode;

import com.evolutionftc.autopilot.AutopilotSystem;
import com.evolutionftc.autopilot.AutopilotTracker;
import com.evolutionftc.autopilot.AutopilotTrackerQP37i;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Marv Mk10 AP Track Test")
public class MarvXAPTrackTest extends OpMode {

    AutopilotTracker tracker;
    AutopilotSystem ap;

    BNO055IMU imu;

    DcMotor qpX;
    DcMotor qpY;

    public void init() {

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        qpX = hardwareMap.dcMotor.get("qpX");
        qpY = hardwareMap.dcMotor.get("qpY");
        qpX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        qpY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        qpX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        qpY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu.initialize(parameters);


        tracker = new AutopilotTrackerQP37i(qpX, qpY, new double[3], 1, imu, 1);
        ap = new AutopilotSystem(tracker, telemetry, hardwareMap.appContext, true);
    }

    public void loop() {
        ap.systemTick();
    }

}
