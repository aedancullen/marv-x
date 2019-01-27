package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Pos2VelTest")
public class Pos2VelTest extends OpMode {

    MarvXICommon marv;

    double targPos;

    public void init() {
        marv = new MarvXICommon(hardwareMap);
    }

    public void loop() {
        marv.telemPoses(telemetry);
        telemetry.update();

        //marv.expand.setPower(marv.pos2vel(0.01, (int)targPos, marv.expand.getCurrentPosition(), 0.5));
        marv.expand.setTargetPosition((int)targPos);
        marv.expand.setPower(0.5);

        if (gamepad1.right_trigger > 0) {
            targPos += gamepad1.right_trigger;
        }
        else if (gamepad1.left_trigger > 0) {
            targPos -= gamepad1.left_trigger;
        }

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
