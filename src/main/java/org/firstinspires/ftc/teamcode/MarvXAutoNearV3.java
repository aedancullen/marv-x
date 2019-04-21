package org.firstinspires.ftc.teamcode;

import com.evolutionftc.autopilot.AutopilotHost;
import com.evolutionftc.autopilot.AutopilotSegment;
import com.evolutionftc.autopilot.AutopilotSystem;
import com.evolutionftc.autopilot.AutopilotTracker;
import com.evolutionftc.autopilot.AutopilotTrackerQP37i;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Near")
public class MarvXAutoNearV3 extends LinearOpMode {

    public static double[] ROBOT_INIT_POSITION = new double[]{-1, 0, 0};
    public static double[] ROBOT_INIT_ATTITUDE = new double[]{-Math.PI / 2, 0, 0};

    MarvXCommonV3 marv;
    MineralFind mineralFind;

    AutopilotHost autopilot;
    AutopilotTracker quadPacer;

    enum HookState{OPEN,LOCK}
    HookState hookState = HookState.LOCK;

    int res = -1;

    public boolean doubleSample = false;

    public void runOpMode() {
        marv = new MarvXCommonV3(hardwareMap, true);
        mineralFind = new MineralFind(hardwareMap);

        marv.imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        marv.imu.initialize(parameters);

        autopilot = new AutopilotHost(telemetry);
        quadPacer = new AutopilotTrackerQP37i(marv.getQuadPacerMotorX(), marv.getQuadPacerMotorY(), MarvConstantsV3.QUADPACER_POS, MarvConstantsV3.QUADPACER_TPU, marv.imu, MarvConstantsV3.QUADPACER_SUBSTEPS);
        ((AutopilotTrackerQP37i)quadPacer).setInverts(false, false);
        autopilot.setCountsToStable(MarvConstantsV3.AP_COUNTS_TO_STABLE);
        autopilot.setNavigationUnitsToStable(MarvConstantsV3.AP_NAV_UNITS_TO_STABLE);
        autopilot.setOrientationUnitsToStable(MarvConstantsV3.AP_ORIENT_UNITS_TO_STABLE);

        mineralFind.detectInitInternal();
        mineralFind.tfod.activate();

        while(!opModeIsActive()) {
            int detect = mineralFind.detectLoopInternal();
            if (detect != -1) {
                res = detect;
            }
            telemetry.addData("Mineral", res);

            telemetry.addData("Press BACK to OPEN","");
            telemetry.addData("Press a JOYSTICK to LOCK","");
            telemetry.update();
            if (isStopRequested()) {
                mineralFind.detectStopInternal();
                return;
            }

            if (gamepad2.back) {
                hookState = HookState.OPEN;
            }
            else if (gamepad2.right_stick_button || gamepad2.left_stick_button) {
                hookState = HookState.LOCK;
            }

            if (hookState == HookState.OPEN) {
                if (marv.expandoVert.getCurrentPosition() > -2000) {
                     marv.expandoVert.setPower(-1);
                }
                else {marv.expandoVert.setPower(0);}
            }
            else if (hookState == HookState.LOCK) {
                if (marv.expandoVertCanLift()) {
                    marv.expandoVert.setPower(1);
                }
                else {marv.expandoVert.setPower(0);}
            }
        }

        mineralFind.detectStopInternal();

        marv.expandoVert.setPower(-1);
        while (marv.dist.getVoltage() < 2.25 && opModeIsActive() && marv.expandoVertCanDrop()) {idle();}
        while (marv.dist.getVoltage() > 2.2 && opModeIsActive() && marv.expandoVertCanDrop()) {idle();}
        int spos = marv.expandoVert.getCurrentPosition();
        while (marv.expandoVert.getCurrentPosition() > spos - MarvConstantsV3.EXPANDO_VERT_EXTRA && marv.expandoVertCanDrop()) {idle();}
        marv.expandoVert.setPower(0);
        autopilot.communicate(quadPacer);
        quadPacer.setRobotPosition(ROBOT_INIT_POSITION);
        quadPacer.setRobotAttitude(ROBOT_INIT_ATTITUDE);

        apGoTo(new double[] {-2.5, 0, 0}, -Math.PI / 2, false, true, false);
        apGoTo(new double[] {-36/3, 15, 0}, -Math.PI / 2, true, true, false);
        apGoTo(new double[] {-36-12, 15, 0}, -Math.PI / 2, true, true, true);
        halt();
        apGoToWithIdle(new double[] {-36-12, 15, 0}, 3*Math.PI / 4, true, false, true);
        halt(); marker();
        if (doubleSample) {
            apGoTo(new double[] {-36-12, 15, 0}, Math.PI, true, false, true);
            halt(); 
            if (sampleRCloseIsFar()) {apGoTo(new double[] {-36-12, 15-7, 0}, Math.PI, false, true, false);halt();}
            sampleRClose();
            apGoToWithIdle(new double[] {-36, 15+12, 0}, -Math.PI / 2 + 0.07, true, true, true);
        }
        else {
            apGoTo(new double[] {-36, 15+12, 0}, -Math.PI / 2 + 0.07, true, true, true);
        }
        halt(); 
        if (sampleLCloseIsFar()) {apGoTo(new double[] {-36+7, 15+12, 0}, -Math.PI / 2 + 0.07, false, true, false);halt();}
        sampleLClose();

        while(opModeIsActive()){idleIntakeAutomation();}

    }

    public void marker() {
        while (marv.expandoHorizL.getCurrentPosition() < MarvConstantsV3.AUTO_MARKER) {
            marv.runIntakeAutomation(0.66, false, true, true, false, false);
        }
        marv.runIntakeAutomation(0, false, false, false, true, false);
        sleep(MarvConstantsV3.EHSM_TRANSFER);
        while (marv.expandoHorizL.getCurrentPosition() > MarvConstantsV3.EXPANDO_HORIZ_DOWN + MarvConstantsV3.UC_EXPANDOHORIZ_BUF) {
            marv.runIntakeAutomation(-0.66, false, false, true, false, false);
        }
        marv.runIntakeAutomation(0, true, false, true, false, false);
    }

    public void sample(int sampleTarget, boolean waitDown) {
        while (marv.expandoHorizL.getCurrentPosition() < sampleTarget) {
            marv.runIntakeAutomation(0.66, false, false, false, false, false);
            marv.runAutomation(false, false);
        }
        if (!waitDown) {marv.runIntakeAutomation(0.66, false, true, false, false, false);}
        else {
            marv.runIntakeAutomation(0, false, true, false, false, false);
            sleep((int)((double)MarvConstantsV3.EHSM_UP/2.0));
            marv.runIntakeAutomation(0.66, false, false, false, false, false);
        }
        while (marv.expandoHorizL.getCurrentPosition() < sampleTarget + MarvConstantsV3.AUTO_SAMPLE_MORE) {
            marv.runIntakeAutomation(0.66, false, false, false, false, false);
            marv.runAutomation(false, false);
        }
        if (sampleTarget >= MarvConstantsV3.EXPANDO_HORIZ_FLYING_LIMIT) {
            while (marv.expandoHorizL.getCurrentPosition() > sampleTarget) {
                marv.runIntakeAutomation(-0.66, false, false, false, false, false);
            }
            marv.runIntakeAutomation(0, true, false, false, false, false);
            sleep((int)((double)MarvConstantsV3.EHSM_UP/2.0));
        }
        while (marv.intakeState != MarvXCommonV3.IntakeState.TRANSFER) {
            marv.runIntakeAutomation(0.0, false, false, false, false, true);
            marv.runAutomation(false, false);
        }
    }

    public void idleIntakeAutomation() {
        marv.runIntakeAutomation(0.0, false, false, false, false, false);
    }

    public void sampleLClose() {
        if (res == 0) {
            sample(MarvConstantsV3.AUTO_SAMPLE_NEAR, true);
        }
        else if (res == 1 || res == -1) {
            sample(MarvConstantsV3.AUTO_SAMPLE_MID, true);
        }
        else if (res == 2) {
            sample(MarvConstantsV3.AUTO_SAMPLE_FAR, true);
        }
    }

    public boolean sampleLCloseIsFar() {
        return res == 2;
    }

    public void sampleRClose() {
        if (res == 2) {
            sample(MarvConstantsV3.AUTO_SAMPLE_NEAR, true);
        }
        else if (res == 1 || res == -1) {
            sample(MarvConstantsV3.AUTO_SAMPLE_MID, true);
        }
        else if (res == 0) {
            sample(MarvConstantsV3.AUTO_SAMPLE_FAR, true);
        }
    }

    public boolean sampleRCloseIsFar() {
        return res == 0;
    }

    public void apGoTo(double[] pos, double hdg, boolean useOrientation, boolean useTranslation, boolean fullStop) {
        apGoTo(pos, hdg, useOrientation, useTranslation, fullStop, false);
    }

    public void apGoToWithIdle(double[] pos, double hdg, boolean useOrientation, boolean useTranslation, boolean fullStop) {
        apGoTo(pos, hdg, useOrientation, useTranslation, fullStop, true);
    }

    public void apGoTo(double[] pos, double hdg, boolean useOrientation, boolean useTranslation, boolean fullStop, boolean doIdle) {
        AutopilotSegment seg = new AutopilotSegment();
        seg.id = "goToSeg";
        seg.success = "n/a";
        seg.fail = "n/a";
        seg.navigationTarget = pos;
        seg.orientationTarget = hdg;
        seg.navigationGain = 0.025;
        seg.orientationGain = 2.00;
        seg.navigationMax = 0.50;
        seg.navigationMin = 0.20;
        seg.orientationMax = 0.50;
        seg.useOrientation = useOrientation;
        seg.useTranslation = useTranslation;
        seg.fullStop = fullStop;

        autopilot.setNavigationTarget(seg);
        autopilot.setNavigationStatus(AutopilotHost.NavigationStatus.RUNNING);

        double [] yxh = null;
        long lastTime = System.currentTimeMillis();

        while (autopilot.getNavigationStatus() == AutopilotHost.NavigationStatus.RUNNING && opModeIsActive()) {
            if (doIdle) {idleIntakeAutomation();}

            if (yxh != null) {
                marv.drive(yxh[0], yxh[0], 1.5*yxh[1], -yxh[2]);
            }
            autopilot.communicate(quadPacer);

            long timeNow = System.currentTimeMillis();
            telemetry.addData("FPS", 1000.0 / (timeNow - lastTime));
            lastTime = timeNow;

            autopilot.telemetryUpdate();
            telemetry.update();
            //AutopilotSystem.visualizerBroadcastRoutine(autopilot);

           yxh = autopilot.navigationTick();
        }
    }

    public void halt() {
        marv.drive(0, 0, 0, 0);
    }
}
