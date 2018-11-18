package org.firstinspires.ftc.teamcode;

import com.evolutionftc.autopilot.AutopilotSystem;
import com.evolutionftc.autopilot.AutopilotTracker;
import com.evolutionftc.autopilot.AutopilotTrackerQP37i;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Marv Mk10 AP Drive Test")
public class MarvXAPDriveTest extends OpMode {

    MarvXCommon marv;

    AutopilotTracker tracker;
    AutopilotSystem ap;

    public void init() {
        marv = new MarvXCommon(hardwareMap);
        marv.setEncoderBehavior(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        marv.setEncoderBehavior(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        marv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tracker = new AutopilotTrackerQP37i(marv.getQuadPacerMotorX(), marv.getQuadPacerMotorY(), new double[3], MarvNavConstants.QUADPACER_TPU, marv.imu, 1);
        ap = new AutopilotSystem(tracker, telemetry, hardwareMap.appContext, true);
    }

    boolean btnALast;
    boolean btnBLast;
    boolean btnXLast;
    boolean btnYLast;

    public void loop() {
        double[] powers = ap.systemTick();
        double powerX = powers[0];
        double powerY = powers[1];
        double rot = powers[2];

        marv.drive(powerY + rot, powerY - rot, powerX);

        if (!btnALast && gamepad1.a) {
            btnALast = true;
            ap.beginPathTravel("patha");
        }
        else if (!btnBLast && gamepad1.b) {
            btnBLast = true;
            ap.beginPathTravel("pathb");
        }
        else if (!btnXLast && gamepad1.x) {
            btnXLast = true;
            ap.beginPathTravel("pathx");
        }
        else if (!btnYLast && gamepad1.y) {
            btnYLast = true;
            ap.beginPathTravel("pathy");
        }

        if (!gamepad1.a) {
            btnALast = false;
        }
        if (!gamepad1.b) {
            btnBLast = false;
        }
        if (!gamepad1.x) {
            btnXLast = false;
        }
        if (!gamepad1.y) {
            btnYLast = false;
        }
    }

}

