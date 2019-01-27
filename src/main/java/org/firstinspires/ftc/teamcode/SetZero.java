package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

@TeleOp(name="SetZero")
public class SetZero extends OpMode {

    Servo rotate;

    public void init() {
        rotate = hardwareMap.servo.get("rotate");
        setServoExtendedRange(rotate, 500, 2500);
    }

    public void setServoExtendedRange(Servo servo, int min, int max) {
        ServoControllerEx controller = (ServoControllerEx) servo.getController();
        int servoPort = servo.getPortNumber();
        PwmControl.PwmRange range = new PwmControl.PwmRange(min, max);
        controller.setServoPwmRange(servoPort, range);
    }

    public void loop() {
        if (gamepad1.a) {
            rotate.setPosition(0);
        }
        else if (gamepad1.b) {
            rotate.setPosition(1);
        }
    }
}
