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
        marv.setIntakeSpeed(gamepad1.right_trigger);
        telemetry.addData("intakeSpeed", gamepad1.right_trigger);
        telemetry.update();
    }

}
