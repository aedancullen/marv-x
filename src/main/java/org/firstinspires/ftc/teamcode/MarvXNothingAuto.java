package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MarvXCommon;

@Autonomous(name="Bootstrap")
public class MarvXNothingAuto extends LinearOpMode {

    MarvXCommon marv;

    public void runOpMode() {
        marv = new MarvXCommon(hardwareMap, false);

        waitForStart();

        marv.expandoVertState = MarvXCommon.ExpandoVertState.SAFE;
        marv.runAllMachinesNoInput();
        while (opModeIsActive() && marv.expandoVertR.isBusy() && marv.expandoVertL.isBusy()) {
            sleep(1);
            marv.runAllMachinesNoInput();
        }
        marv.expandoHorizState = MarvXCommon.ExpandoHorizState.STROBE_DUMP;
        marv.boxLiftState = MarvXCommon.BoxLiftState.DOWN;
        marv.expandoVertState = MarvXCommon.ExpandoVertState.DOWN;
        marv.runAllMachinesNoInput();
        while (opModeIsActive() && marv.expandoHorizState != MarvXCommon.ExpandoHorizState.MANUAL) {
            sleep(1);
            marv.runAllMachinesNoInput();
        }
        marv.intakeState = MarvXCommon.IntakeState.UP_NEUTRAL;
        marv.runAllMachinesNoInput();
        while (opModeIsActive()) {
            marv.runAllMachinesNoInput();
        }
    }

}