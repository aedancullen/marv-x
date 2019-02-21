package org.firstinspires.ftc.teamcode;

import com.evolutionftc.autopilot.AutopilotHost;
import com.evolutionftc.autopilot.AutopilotSegment;
import com.evolutionftc.autopilot.AutopilotSystem;
import com.evolutionftc.autopilot.AutopilotTracker;
import com.evolutionftc.autopilot.AutopilotTrackerEnc;
import com.evolutionftc.autopilot.AutopilotTrackerEncMec;
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

        marv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        marv.imu = hardwareMap.get(BNO055IMU.class, "realImu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        marv.imu.initialize(parameters);

        marv.vertSwing.setPosition(0.5);

        ap = new AutopilotHost(telemetry);
        qpTracker = new AutopilotTrackerQP37i(marv.getQuadPacerMotorX(), marv.getQuadPacerMotorY(), new double[3], 150, marv.imu, 1);
        ((AutopilotTrackerQP37i)qpTracker).setInverts(false, true);
        //qpTracker = new AutopilotTrackerEncMec(marv.fl, marv.fr, marv.bl, marv.br, 500, marv.imu, 1);
        ap.setCountsToStable(5);
        ap.setNavigationUnitsToStable(1);
        ap.setOrientationUnitsToStable(0.05);

        int res = -1;

        /*mineralFind.detectInit();
        while (!opModeIsActive()) {
            int detect = mineralFind.detectLoop();
            if (detect != -1) {
                res = detect;
            }
            telemetry.addData("res", res);
            telemetry.update();
            if (isStopRequested()) {
                mineralFind.detectStop();
                return;
            }
        }

        mineralFind.detectStop();*/
        waitForStart();


        /*while (opModeIsActive()) {
            telemetry.addData("MF", res);

            ap.communicate(qpTracker);

            ap.telemetryUpdate();
            telemetry.update();
            AutopilotSystem.visualizerBroadcastRoutine(ap);
        }*/

        //mineralFind.detectStop();

        /*apGoTo(new double[] {0, 6, 0}, 0, true);
        //apGoTo(new double[] {0, 23, 0}, Math.PI / 4, true); // C
        apGoTo(new double[] {16, 23, 0}, Math.PI / 4, true); // R
        //apGoTo(new double[] {-16, 23, 0}, Math.PI / 4, true); // L

        apGoTo(new double[] {0, 14, 0}, Math.PI / 2, true); // clear

        apGoTo(new double[] {-43, 14, 0}, Math.PI / 2, true); // across
        apGoTo(new double[] {-43, 14, 0}, Math.PI / 4, true); // across*/

        apGoTo(new double[] {0, 12, 0}, 0, true);
        apGoTo(new double[] {0, 12, 0}, Math.PI / 2, true);


        apGoTo(new double[] {17, 25, 0}, Math.PI / 2, true); // R
        //apGoTo(new double[] {0, 25, 0}, Math.PI / 2, true); // C
        //apGoTo(new double[] {-17, 25, 0}, Math.PI / 2, true); // L

        apGoTo(new double[] {0, 12, 0}, Math.PI / 2, true);
        apGoTo(new double[] {-43, 16, 0}, Math.PI / 2, true); // across
        apGoTo(new double[] {-43, 16, 0}, Math.PI / 4, true); // across



        while(opModeIsActive()) {sleep(1);}


    }

    public void apGoTo(double[] pos, double hdg, boolean useOrientation) {
        AutopilotSegment seg = new AutopilotSegment();
        seg.id = "goToSeg";
        seg.success = "n/a";
        seg.fail = "n/a";
        seg.navigationTarget = pos;
        seg.orientationTarget = hdg;
        seg.navigationGain = 0.035; // something
        seg.orientationGain = 1.5; // something
        seg.navigationMax = 0.35;
        seg.navigationMin = 0.25;
        seg.orientationMax = 0.35;
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
