package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Autonomous(name="MarvXNothingAutoV2")
public class MarvXNothingAutoV2 extends LinearOpMode {

    MarvXCommonV2 marv;

    public void runOpMode() {
        marv = new MarvXCommonV2(hardwareMap, false);

        waitForStart();

        marv.expandoVertR.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_SAFE);
        marv.expandoVertL.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_SAFE);
        marv.expandoVertR.setPower(MarvConstantsV2.EXPANDO_VERT_TOSAFE_SPEED);
        marv.expandoVertL.setPower(MarvConstantsV2.EXPANDO_VERT_TOSAFE_SPEED);

        marv.vertLiftL.setPosition(MarvConstantsV2.VERT_LIFT_SAFE);
        marv.vertLiftR.setPosition(MarvConstantsV2.VERT_LIFT_SAFE);

        while (marv.expandoVertR.isBusy() || marv.expandoVertL.isBusy()) {sleep(1);}

        marv.expandoHorizL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        marv.expandoHorizL.setTargetPosition(MarvConstantsV2.EXPANDO_HORIZ_SAFE);
        marv.expandoHorizL.setPower(0.5);
        marv.horizLiftL.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);
        marv.horizLiftR.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);

        while (marv.expandoHorizL.isBusy()) {sleep(1);}

        sleep(250);

        marv.vertLiftL.setPosition(MarvConstantsV2.VERT_LIFT_DOWN);
        marv.vertLiftR.setPosition(MarvConstantsV2.VERT_LIFT_DOWN);
        marv.expandoVertR.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_DOWN);
        marv.expandoVertL.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_DOWN);
        marv.expandoVertR.setPower(MarvConstantsV2.EXPANDO_VERT_TODOWN_SPEED);
        marv.expandoVertL.setPower(MarvConstantsV2.EXPANDO_VERT_TODOWN_SPEED);


        while (marv.expandoVertR.isBusy() || marv.expandoVertL.isBusy()) {sleep(1);}

    }
}
