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

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    BNO055IMU imu;

    public DcMotor getQuadPacerMotorX() {
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return fr;
    }

    public DcMotor getQuadPacerMotorY() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return fl;
    }

    public void runOpMode() {
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu.initialize(parameters);

        ap = new AutopilotHost(telemetry);
        qpTracker = new AutopilotTrackerQP37i(getQuadPacerMotorX(), getQuadPacerMotorY(), new double[3], 158, imu, 1);
        ((AutopilotTrackerQP37i)qpTracker).setInverts(true, false);
        ap.setCountsToStable(5);

        waitForStart();

        while (opModeIsActive()) {
            ap.communicate(qpTracker);

            ap.telemetryUpdate();
            telemetry.update();
            AutopilotSystem.visualizerBroadcastRoutine(ap);
        }

        //apGoTo(new double[] {0, 12, 0}, 0, false);


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

            ap.telemetryUpdate();
            telemetry.update();
            AutopilotSystem.visualizerBroadcastRoutine(ap);

            double[] yxh = ap.navigationTick();
            drive(yxh[0], yxh[0], yxh[1], -yxh[2]);
        }
        drive(0, 0, 0, 0);
    }

    public void drive(
            double vertL,
            double vertR,
            double horiz,
            double rot
    ) {

        double flp = vertL + rot + horiz;
        fl.setPower(Math.max(Math.min(flp, 1), -1));
        double frp = vertR - rot - horiz;
        fr.setPower(Math.max(Math.min(frp, 1), -1));
        double blp = vertL + rot - horiz;
        bl.setPower(Math.max(Math.min(blp, 1), -1));
        double brp = vertR - rot + horiz;
        br.setPower(Math.max(Math.min(brp, 1), -1));
    }

}
