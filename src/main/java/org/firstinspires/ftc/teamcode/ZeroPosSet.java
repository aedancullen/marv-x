package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

@TeleOp(name="ZPS")
public class ZeroPosSet extends OpMode {

    Servo horizLiftL;
    Servo horizLiftR;

    Servo diagDrop;
    Servo diagSwop;

    public void init() {
        horizLiftL = hardwareMap.servo.get("horizLiftL");
        horizLiftL.setDirection(Servo.Direction.REVERSE);

        horizLiftR = hardwareMap.servo.get("horizLiftR");

        diagDrop = hardwareMap.servo.get("drop");
        diagSwop = hardwareMap.servo.get("swop");

        setServoExtendedRange(horizLiftL, 500, 2500);
        setServoExtendedRange(horizLiftR, 500, 2500);
        setServoExtendedRange(diagDrop, 500, 2500);
        setServoExtendedRange(diagSwop, 500, 2500);

        horizLiftL.setPosition(0.14);
        horizLiftR.setPosition(0.14);
        diagDrop.setPosition(0.5);
    }

    public void loop() {
        if (!gamepad2.a){
            horizLiftL.setPosition(0.14);
            horizLiftR.setPosition(0.14);
        }
        else {
            horizLiftL.setPosition(0.5);
            horizLiftR.setPosition(0.5);
        }
    }

    public void setServoExtendedRange(Servo servo, int min, int max) {
        ServoControllerEx controller = (ServoControllerEx) servo.getController();
        int servoPort = servo.getPortNumber();
        PwmControl.PwmRange range = new PwmControl.PwmRange(min, max);
        controller.setServoPwmRange(servoPort, range);
    }
}
