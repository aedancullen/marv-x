package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ExpandoDiagTest")
public class ExpandoDiagTest extends OpMode {

    MarvXCommonV3 marv;

    public void init() {
        marv = new MarvXCommonV3(hardwareMap, true);
    }

    public void loop() {

        telemetry.addData("horizL", marv.expandoHorizL.getCurrentPosition());
        telemetry.addData("horizR", marv.expandoHorizR.getCurrentPosition());

        if (gamepad2.a) {

        marv.expandoDiag.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

        telemetry.addData("Diag position", marv.expandoDiag.getCurrentPosition());


        }
        else if (gamepad2.b) {
        marv.drop.setPosition(0.5 + (gamepad2.right_trigger - gamepad2.left_trigger) / 2);

        telemetry.addData("Drop position", marv.drop.getPosition());

        }
        else if (gamepad2.x) {
        marv.swop.setPosition(0.5 + (gamepad2.right_trigger - gamepad2.left_trigger) / 2);
        telemetry.addData("Swap position", marv.swop.getPosition());

        }
        else if (gamepad2.y) {
            marv.horizLiftL.setPosition(MarvConstantsV3.HORIZ_LIFT_DOWN + gamepad2.right_trigger);
            marv.horizLiftR.setPosition(MarvConstantsV3.HORIZ_LIFT_DOWN + gamepad2.right_trigger);
            telemetry.addData("Lift position", MarvConstantsV3.HORIZ_LIFT_DOWN + gamepad2.right_trigger);
        }

        if (gamepad2.back) {
            marv.horizSpinL.setPower(MarvConstantsV3.UC_HORIZSPIN_TRANSFER);
            marv.horizSpinR.setPower(MarvConstantsV3.UC_HORIZSPIN_TRANSFER);
        }
        else {
            marv.horizSpinL.setPower(0);
            marv.horizSpinR.setPower(0);
        }
        telemetry.update();
    }

}
