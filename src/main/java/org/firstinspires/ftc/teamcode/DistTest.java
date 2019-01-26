package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="DistTest")
public class DistTest extends OpMode {

    Rev2mDistanceSensor dist;

    public void init() {
        dist = hardwareMap.get(Rev2mDistanceSensor.class, "dist");
    }

    public void loop() {
        telemetry.addData("dist inches", dist.getDistance(DistanceUnit.INCH));
    }

}
