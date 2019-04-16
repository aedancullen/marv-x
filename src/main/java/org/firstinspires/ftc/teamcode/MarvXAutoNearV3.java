package org.firstinspires.ftc.teamcode;

import com.evolutionftc.autopilot.AutopilotHost;
import com.evolutionftc.autopilot.AutopilotSegment;
import com.evolutionftc.autopilot.AutopilotSystem;
import com.evolutionftc.autopilot.AutopilotTracker;
import com.evolutionftc.autopilot.AutopilotTrackerQP37i;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class MarvXAutoNearV3 extends LinearOpMode {

    public static double[] ROBOT_INIT_POSITION;
    public static double[] ROBOT_INIT_ATTITUDE = new double[]{-Math.PI/2, 0, 0};

    MarvXCommonV3 marv;
    MineralFind mineralFind;

    AutopilotHost autopilot;
    AutopilotTracker quadPacer;

    int res;

    public void runOpMode() {
        marv = new MarvXCommonV3(hardwareMap, true);
        mineralFind = new MineralFind(hardwareMap);

        marv.imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        marv.imu.initialize(parameters);

        autopilot = new AutopilotHost(telemetry);
        quadPacer = new AutopilotTrackerQP37i(marv.getQuadPacerMotorX(), marv.getQuadPacerMotorY(), MarvConstantsV2.QUADPACER_POS, MarvConstantsV2.QUADPACER_TPU, marv.imu, 1);
        ((AutopilotTrackerQP37i)quadPacer).setInverts(false, false);
        autopilot.setCountsToStable(MarvConstantsV2.AP_COUNTS_TO_STABLE);
        autopilot.setNavigationUnitsToStable(MarvConstantsV2.AP_NAV_UNITS_TO_STABLE);
        autopilot.setOrientationUnitsToStable(MarvConstantsV2.AP_ORIENT_UNITS_TO_STABLE);

        mineralFind.detectInitInternal();
        mineralFind.tfod.activate();

        while(!opModeIsActive()) {
            int detect = mineralFind.detectLoopInternal();
            if (detect != -1) {
                res = detect;
            }
            telemetry.addData("Mineral", res);
            telemetry.update();
            if (isStopRequested()) {
                mineralFind.detectStopInternal();
                return;
            }
        }

        mineralFind.detectStopInternal();

        // drop routine aqui
        quadPacer.setRobotPosition(ROBOT_INIT_POSITION);
        quadPacer.setRobotAttitude(ROBOT_INIT_ATTITUDE);

    }

    public void apGoTo(double[] pos, double hdg, boolean useOrientation, boolean useTranslation, boolean fullStop) {
        AutopilotSegment seg = new AutopilotSegment();
        seg.id = "goToSeg";
        seg.success = "n/a";
        seg.fail = "n/a";
        seg.navigationTarget = pos;
        seg.orientationTarget = hdg;
        seg.navigationGain = 0.035; // something
        seg.orientationGain = 2.35; // something
        seg.navigationMax = 0.35;
        seg.navigationMin = 0.25;
        seg.orientationMax = 0.30;
        seg.useOrientation = useOrientation;
        seg.useTranslation = useTranslation;
        seg.fullStop = fullStop;

        autopilot.setNavigationTarget(seg);
        autopilot.setNavigationStatus(AutopilotHost.NavigationStatus.RUNNING);

        double [] yxh = null;

        while (autopilot.getNavigationStatus() == AutopilotHost.NavigationStatus.RUNNING && opModeIsActive()) {
            if (yxh != null) {
                marv.drive(yxh[0], yxh[0], yxh[1], -yxh[2]);
            }
            autopilot.communicate(quadPacer);

            //autopilot.telemetryUpdate();
            //telemetry.update();
            //AutopilotSystem.visualizerBroadcastRoutine(autopilot);

           yxh = autopilot.navigationTick();
        }
    }

    public void halt() {
        marv.drive(0, 0, 0, 0);
    }
}
