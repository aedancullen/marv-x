package org.firstinspires.ftc.teamcode;

import com.evolutionftc.autopilot.AutopilotHost;
import com.evolutionftc.autopilot.AutopilotSegment;
import com.evolutionftc.autopilot.AutopilotSystem;
import com.evolutionftc.autopilot.AutopilotTracker;
import com.evolutionftc.autopilot.AutopilotTrackerQP37i;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.sun.tools.javac.code.Types;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="AutoNear V2")
public class MarvXAutoNearV2 extends LinearOpMode {

    MarvXCommonV2 marv;
    MineralFind mineralFind;

    AutopilotHost autopilot;
    AutopilotTracker quadPacer;

    public void runOpMode() {
        marv = new MarvXCommonV2(hardwareMap, false);
        mineralFind = new MineralFind(hardwareMap);

        marv.imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        marv.imu.initialize(parameters);

        autopilot = new AutopilotHost(telemetry);
        quadPacer = new AutopilotTrackerQP37i(marv.getQuadPacerMotorX(), marv.getQuadPacerMotorY(), MarvConstantsV2.QUADPACER_POS, MarvConstantsV2.QUADPACER_TPU, marv.imu, 1);
        ((AutopilotTrackerQP37i)quadPacer).setInverts(false, true);
        autopilot.setCountsToStable(MarvConstantsV2.AP_COUNTS_TO_STABLE);
        autopilot.setNavigationUnitsToStable(MarvConstantsV2.AP_NAV_UNITS_TO_STABLE);
        autopilot.setOrientationUnitsToStable(MarvConstantsV2.AP_ORIENT_UNITS_TO_STABLE);


        marv.vertLatch.setPosition(MarvConstantsV2.VERT_LATCH_LOCKED);
        marv.tmd.setPosition(MarvConstantsV2.TMD_IN);

        marv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        int res = -1;

        mineralFind.detectInitInternal();
        waitForStart();

        //while (opModeIsActive()) {sleep(1);}

        marv.horizLiftL.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);
        marv.horizLiftR.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);

        marv.vertLiftL.setPosition(MarvConstantsV2.VERT_LIFT_DOWN);
        marv.vertLiftR.setPosition(MarvConstantsV2.VERT_LIFT_DOWN);
        marv.vertSwing.setPosition(MarvConstantsV2.VERT_SWING_CENTER);
        marv.vertSpin.setPosition(MarvConstantsV2.VERT_SPIN_NEUTRAL);

        marv.expandoVertL.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_DOWN);
        marv.expandoVertR.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_DOWN);
        marv.expandoVertL.setPower(1);
        marv.expandoVertR.setPower(1);

        marv.vertLatch.setPosition(MarvConstantsV2.VERT_LATCH_OPEN);

        long timeStart = System.currentTimeMillis();
        while (System.currentTimeMillis() - timeStart < 1000 && opModeIsActive()) {sleep(1);}

        marv.expandoVertL.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_SAFE);
        marv.expandoVertR.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_SAFE);
        marv.expandoVertL.setPower(MarvConstantsV2.EXPANDO_VERT_TOSAFE_SPEED);
        marv.expandoVertR.setPower(MarvConstantsV2.EXPANDO_VERT_TOSAFE_SPEED);

        while (marv.altitude.getDistance(DistanceUnit.INCH) > 2.5 && opModeIsActive()) {sleep(1);}

        marv.expandoVertL.setTargetPosition(marv.expandoVertL.getCurrentPosition() + MarvConstantsV2.EXPANDO_VERT_2IN);
        marv.expandoVertR.setTargetPosition(marv.expandoVertR.getCurrentPosition() + MarvConstantsV2.EXPANDO_VERT_2IN);


        while (marv.expandoVertL.isBusy() && marv.expandoVertR.isBusy()) {sleep(1);}

        marv.expandoVertL.setPower(0);
        marv.expandoVertR.setPower(0);

        marv.setEncoderBehavior(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        marv.setEncoderBehavior(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        apGoTo(new double[] {-2.5, 0, 0}, 0, false);

        //while (opModeIsActive()) {sleep(1);}

        marv.expandoVertL.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_DOWN);
        marv.expandoVertR.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_DOWN);
        marv.expandoVertL.setPower(MarvConstantsV2.EXPANDO_VERT_TODOWN_SPEED);
        marv.expandoVertR.setPower(MarvConstantsV2.EXPANDO_VERT_TODOWN_SPEED);

        //while (marv.expandoVertL.isBusy() && marv.expandoVertR.isBusy()) {sleep(1);}

        //marv.expandoVertL.setPower(0);
        //marv.expandoVertR.setPower(0);

        apGoTo(new double[] {0, 8, 0}, 0, true);

        apGoTo(new double[] {0, 8, 0}, -Math.PI / 2, true, false); // Camera

        long detectStartTime = System.currentTimeMillis();
        mineralFind.tfod.activate();

        while (res == -1 && System.currentTimeMillis() - detectStartTime < 3000) {
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

        marv.expandoVertL.setPower(0);
        marv.expandoVertR.setPower(0);

        mineralFind.detectStopInternal();

        if (res == 1) {
            apGoTo(new double[]{0, 22, 0}, -Math.PI / 4, true); // C
        }
        else if (res == 2) {
            apGoTo(new double[]{16, 22, 0}, -Math.PI / 4, true); // R
        }
        else if (res == 0 || res == -1) {
            apGoTo(new double[] {-16, 22, 0}, -Math.PI / 4, true); // L
        }

        if (res != 0 && res != -1) {
            apGoTo(new double[]{7.5, 15, 0}, -Math.PI / 2, true); // clear not on L
            apGoTo(new double[]{-40.5, 15, 0}, -Math.PI / 2, true); // across not on L
        }
        apGoTo(new double[] {-46.5, 15, 0}, -Math.PI / 4, true); // across

        apGoTo(new double[] {-70.5, -9, 0}, -Math.PI / 4, true); // depot

        apGoTo(new double[] {-70.5, -19, 0}, 0, true); // position

        marv.tmd.setPosition(MarvConstantsV2.TMD_OUT);

        //while (opModeIsActive()) {sleep(1);}

        sleep(500);

        apGoTo(new double[] {-70.5, -9, 0}, -Math.PI / 4, true); // back
        apGoTo(new double[] {-33, 33, 0}, -Math.PI / 4, true); // crater

        marv.tmd.setPosition(MarvConstantsV2.TMD_IN);
    }

    public void apGoTo(double[] pos, double hdg, boolean useOrientation) {
        apGoTo(pos, hdg, useOrientation, true);
    }

    public void apGoTo(double[] pos, double hdg, boolean useOrientation, boolean useTranslation) {
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

        autopilot.setNavigationTarget(seg);
        autopilot.setNavigationStatus(AutopilotHost.NavigationStatus.RUNNING);

        while (autopilot.getNavigationStatus() == AutopilotHost.NavigationStatus.RUNNING && opModeIsActive()) {
            autopilot.communicate(quadPacer);

            autopilot.telemetryUpdate();
            telemetry.update();
            AutopilotSystem.visualizerBroadcastRoutine(autopilot);

            double[] yxh = autopilot.navigationTick();
            marv.drive(yxh[0], yxh[0], yxh[1], -yxh[2]);
        }
        marv.drive(0, 0, 0, 0);
    }

}
