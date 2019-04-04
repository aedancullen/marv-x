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

        if (gamepad2.a) {

        marv.expandoDiag.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

        telemetry.addData("Diag position", marv.expandoDiag.getCurrentPosition());
        telemetry.update();

        }
        else if (gamepad2.b) {
        marv.drop.setPosition(0.5 + (gamepad2.right_trigger - gamepad2.left_trigger) / 2);

        telemetry.addData("Drop position", marv.drop.getPosition());
        telemetry.update();
        }
        else if (gamepad.x) {
        marv.swop.setPosition(0.5 + (gamepad2.right_trigger - gamepad2.left_trigger) / 2);
        telemetry.addData("Swap position", marv.swop.getPosition());
        telemetry.update();
        }
    }

}
