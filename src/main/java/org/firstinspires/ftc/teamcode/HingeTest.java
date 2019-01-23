package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="HingeTest")
public class HingeTest extends OpMode {

    public MarvXICommon marv;

    public void init() {
        marv = new MarvXICommon(hardwareMap);
    }

    public void loop() {
        if (gamepad1.a) {
            marv.setHingeSpeed(0.1);
        }
        else if (gamepad1.b) {
            marv.setHingeSpeed(-0.1);
        }
        else if (gamepad1.x) {
            marv.setHingeSpeed(0.5);
        }
        else if (gamepad1.y) {
            marv.setHingeSpeed(-0.5);
        }
        else {
            marv.setHingeSpeed(0);
        }
    }

}
