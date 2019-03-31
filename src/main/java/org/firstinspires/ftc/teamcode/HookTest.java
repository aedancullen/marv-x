package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="HookTest")
public class HookTest extends OpMode {

    DcMotor hook;

    public void init() {
        hook = hardwareMap.dcMotor.get("expandoHorizR");
        hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void loop() {
        if (gamepad2.right_bumper) {
            hook.setPower(1);
        }
        else if (gamepad2.left_bumper) {
            hook.setPower(-1);
        }
        else {
            hook.setPower(0);
        }
    }

}
