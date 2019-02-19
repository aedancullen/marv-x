package org.firstinspires.ftc.teamcode;

import com.evolutionftc.autopilot.AutopilotHost;
import com.evolutionftc.autopilot.AutopilotTracker;
import com.evolutionftc.autopilot.AutopilotTrackerQP37i;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="AutoNear")
public class MarvXAutoNear extends LinearOpMode {

    MarvXCommonV2 marv;
    MineralFind mineralFind;

    AutopilotHost autopilot;
    AutopilotTracker quadPacer;

    public void runOpMode() {
        marv = new MarvXCommonV2(hardwareMap, false);
        mineralFind = new MineralFind(hardwareMap);

        marv.imu = hardwareMap.get(BNO055IMU.class, "realImu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        marv.imu.initialize(parameters);

        autopilot = new AutopilotHost(telemetry);
        quadPacer = new AutopilotTrackerQP37i(marv.getQuadPacerMotorX(), marv.getQuadPacerMotorY(), new double[3], MarvConstantsV2.QUADPACER_TPU, marv.imu, 1);
        ((AutopilotTrackerQP37i)quadPacer).setInverts(false, true);
        autopilot.setCountsToStable(MarvConstantsV2.AP_COUNTS_TO_STABLE);
        autopilot.setNavigationUnitsToStable(MarvConstantsV2.AP_NAV_UNITS_TO_STABLE);
        autopilot.setOrientationUnitsToStable(MarvConstantsV2.AP_ORIENT_UNITS_TO_STABLE);


        marv.vertLatch.setPosition(MarvConstantsV2.VERT_LATCH_LOCKED);


        int res = -1;

        mineralFind.detectInit();
        while (!opModeIsActive()) {
            int detect = mineralFind.detectLoop();
            if (detect != -1) {
                res = detect;
            }
            telemetry.addData("Mineral", res);
            telemetry.update();
            if (isStopRequested()) {
                mineralFind.detectStop();
                return;
            }
        }
        //waitForStart();
        mineralFind.detectStop();
        marv.horizLiftL.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);
        marv.horizLiftR.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);

        marv.expandoVertL.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_DOWN);
        marv.expandoVertR.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_DOWN);
        marv.expandoVertL.setPower(1);
        marv.expandoVertR.setPower(1);

        marv.vertLatch.setPosition(MarvConstantsV2.VERT_LATCH_OPEN);

        long timeStart = System.currentTimeMillis();
        while (System.currentTimeMillis() - timeStart < 750 && opModeIsActive()) {sleep(1);}

        marv.expandoVertL.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_SAFE);
        marv.expandoVertR.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_SAFE);
        marv.expandoVertL.setPower(MarvConstantsV2.EXPANDO_VERT_TOSAFE_SPEED);
        marv.expandoVertR.setPower(MarvConstantsV2.EXPANDO_VERT_TOSAFE_SPEED);

        while (marv.altitude.getDistance(DistanceUnit.INCH) > 2.5 && opModeIsActive()) {sleep(1);}

        marv.expandoVertL.setTargetPosition(marv.expandoVertL.getCurrentPosition() + MarvConstantsV2.EXPANDO_VERT_2IN);
        marv.expandoVertR.setTargetPosition(marv.expandoVertR.getCurrentPosition() + MarvConstantsV2.EXPANDO_VERT_2IN);


        while (opModeIsActive()) {
            telemetry.addData("l", marv.expandoVertL.getCurrentPosition());
            telemetry.addData("r", marv.expandoVertR.getCurrentPosition());
            telemetry.update();
        }
    }

}
