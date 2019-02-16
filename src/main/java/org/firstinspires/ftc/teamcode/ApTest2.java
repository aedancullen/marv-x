package org.firstinspires.ftc.teamcode;

import com.evolutionftc.autopilot.AutopilotHost;
import com.evolutionftc.autopilot.AutopilotSegment;
import com.evolutionftc.autopilot.AutopilotSystem;
import com.evolutionftc.autopilot.AutopilotTracker;
import com.evolutionftc.autopilot.AutopilotTrackerQP37i;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="APTest2")
public class ApTest2 extends LinearOpMode {

    AutopilotHost ap;
    AutopilotTracker qpTracker;

    MarvXCommonV2 marv;

    MineralFind mineralFind;


    public void runOpMode() {

        mineralFind = new MineralFind(hardwareMap);

        marv = new MarvXCommonV2(hardwareMap, false);

        marv.imu = hardwareMap.get(BNO055IMU.class, "realImu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        marv.imu.initialize(parameters);

        ap = new AutopilotHost(telemetry);
        qpTracker = new AutopilotTrackerQP37i(marv.getQuadPacerMotorX(), marv.getQuadPacerMotorY(), new double[3], 158, marv.imu, 1);
        ((AutopilotTrackerQP37i)qpTracker).setInverts(false, true);
        ap.setCountsToStable(5);
        ap.setNavigationUnitsToStable(1);
        ap.setOrientationUnitsToStable(0.05);

        int res = -1;

        mineralFind.detectInit();
        while (!opModeIsActive()) {
            int detect = mineralFind.detectLoop();
            if (detect != -1) {
                res = detect;
            }
        }
        //waitForStart();

        mineralFind.detectStop();


        while (opModeIsActive()) {
            telemetry.addData("MF", res);

            ap.communicate(qpTracker);

            ap.telemetryUpdate();
            telemetry.update();
            AutopilotSystem.visualizerBroadcastRoutine(ap);
        }

        //mineralFind.detectStop();

        //apGoTo(new double[] {0, 12, 0}, 0, false);


    }

    public void apGoTo(double[] pos, double hdg, boolean useOrientation) {
        AutopilotSegment seg = new AutopilotSegment();
        seg.id = "goToSeg";
        seg.success = "n/a";
        seg.fail = "n/a";
        seg.navigationTarget = pos;
        seg.orientationTarget = hdg;
        seg.navigationGain = 0.5; // something
        seg.orientationGain = 0.5; // something
        seg.navigationMax = 0.5;
        seg.navigationMin = 0;
        seg.orientationMax = 0.5;
        seg.useOrientation = useOrientation;

        ap.setNavigationTarget(seg);
        ap.setNavigationStatus(AutopilotHost.NavigationStatus.RUNNING);

        while (ap.getNavigationStatus() == AutopilotHost.NavigationStatus.RUNNING && opModeIsActive()) {
            ap.communicate(qpTracker);

            ap.telemetryUpdate();
            telemetry.update();
            AutopilotSystem.visualizerBroadcastRoutine(ap);

            double[] yxh = ap.navigationTick();
            marv.drive(yxh[0], yxh[0], yxh[1], -yxh[2]);
        }
        marv.drive(0, 0, 0, 0);
    }

}
