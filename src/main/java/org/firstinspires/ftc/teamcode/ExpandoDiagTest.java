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

        marv.expandoDiag.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

        telemetry.addData("Position", marv.expandoDiag.getCurrentPosition());
        telemetry.update();
    }

}
