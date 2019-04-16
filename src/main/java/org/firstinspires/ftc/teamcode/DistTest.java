package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@TeleOp(name="DistTest")
public class DistTest extends OpMode {

    AnalogInput dist;

    MarvXCommonV2 marv;

    public void init() {
        dist = hardwareMap.analogInput.get("dist");
        //marv = new MarvXCommonV2(hardwareMap, false);
    }

    public void loop() {
        telemetry.addData("dist", dist.getVoltage());
        //marv.horizLiftL.setPosition(gamepad2.right_trigger);
        //marv.horizLiftR.setPosition(gamepad2.right_trigger);
        //marv.vertSwing.setPosition(gamepad2.right_trigger / 4.0 + 0.497);
        telemetry.update();
    }

}
