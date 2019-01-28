package org.firstinspires.ftc.teamcode;

import com.evolutionftc.autopilot.AutopilotHost;
import com.evolutionftc.autopilot.AutopilotSegment;
import com.evolutionftc.autopilot.AutopilotTracker;
import com.evolutionftc.autopilot.AutopilotTrackerQP37i;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class MarvXIFarsideAuto extends LinearOpMode {

    MarvXICommon marv;

    AutopilotHost ap;
    AutopilotTracker qpTracker;

    public void runOpMode() {
        marv = new MarvXICommon(hardwareMap);
        ap = new AutopilotHost(telemetry);
        qpTracker = new AutopilotTrackerQP37i(marv.getQuadPacerMotorX(), marv.getQuadPacerMotorY(), new double[3], MarvXICommon.AP_TICKS_PER_INCH, marv.imu, 1);
        ap.setCountsToStable(MarvXICommon.AP_COUNTS_TO_STABLE);
        waitForStart();
    }

    public void apGoTo(double[] pos, double hdg, boolean useOrientation) {
        AutopilotSegment seg = new AutopilotSegment();
        seg.id = "goToSeg";
        seg.success = "n/a";
        seg.fail = "n/a";
        seg.navigationTarget = pos;
        seg.orientationTarget = hdg;
        seg.navigationGain = MarvXICommon.NAV_GAIN_PER_INCH;
        seg.orientationGain = MarvXICommon.ORIENT_GAIN_PER_INCH;
        seg.navigationMax = 0.5;
        seg.navigationMin = 0;
        seg.orientationMax = 0.5;
        seg.useOrientation = useOrientation;

        ap.setNavigationTarget(seg);
        ap.setNavigationStatus(AutopilotHost.NavigationStatus.RUNNING);

        while (ap.getNavigationStatus() == AutopilotHost.NavigationStatus.RUNNING && opModeIsActive()) {
            ap.communicate(qpTracker);
            double[] yxh = ap.navigationTick();
            marv.drive(yxh[0], yxh[0], yxh[1], -yxh[2]);
        }
        marv.drive(0, 0, 0, 0);

    }


}
