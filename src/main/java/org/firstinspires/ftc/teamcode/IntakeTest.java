package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="IntakeTest")
public class IntakeTest extends OpMode {

    public MarvXICommon marv;

    public void init() {
        marv = new MarvXICommon(hardwareMap);
    }

    public void loop() {
        if (gamepad1.a) {
            marv.setIntakeSpeed(0.1);
        }
        else if (gamepad1.b) {
            marv.setIntakeSpeed(-0.1);
        }
        else if (gamepad1.x) {
            marv.setIntakeSpeed(0.5);
        }
        else if (gamepad1.y) {
            marv.setIntakeSpeed(-0.5);
        }
        else {
            marv.setIntakeSpeed(0);
        }
    }

}
