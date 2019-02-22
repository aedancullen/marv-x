package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="DistTest")
public class DistTest extends OpMode {

    Rev2mDistanceSensor dist;

    MarvXCommonV2 marv;

    public void init() {
        dist = hardwareMap.get(Rev2mDistanceSensor.class, "dist");
        marv = new MarvXCommonV2(hardwareMap, false);
        marv.vertSwing.setPosition(0.5);
        marv.expandoHorizL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        marv.expandoVertL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        marv.expandoVertR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void loop() {
        telemetry.addData("dist inches", dist.getDistance(DistanceUnit.INCH));
        //marv.horizLiftL.setPosition(gamepad2.right_trigger);
        //marv.horizLiftR.setPosition(gamepad2.right_trigger);
        //marv.vertSwing.setPosition(gamepad2.right_trigger / 4.0 + 0.497);
        marv.tmd.setPosition(gamepad2.right_trigger);
        telemetry.addData("trig", gamepad2.right_trigger);
        telemetry.addData("expando", marv.expandoHorizL.getCurrentPosition());
        telemetry.update();
    }

}
