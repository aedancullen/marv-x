package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Marv Mk10 UserTests")
public class MarvXUserControl extends OpMode {

    Servo testServo1;
    Servo testServo2;
    Servo testServo3;
    Servo testServo4;
    DcMotor testMotor1;
    DcMotor testMotor2;

    public void init(){
        testServo1 = hardwareMap.servo.get("testServo1");
        setServoExtendedRange(testServo1, 500, 2500);
        testServo2 = hardwareMap.servo.get("testServo2");
        setServoExtendedRange(testServo2, 500, 2500);
        testServo2.setDirection(Servo.Direction.REVERSE);
        testServo3 = hardwareMap.servo.get("testServo3");
        testServo4 = hardwareMap.servo.get("testServo4");
        testServo4.setDirection(Servo.Direction.REVERSE);
        testMotor1 = hardwareMap.dcMotor.get("testMotor1");
        testMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        testMotor2 = hardwareMap.dcMotor.get("testMotor2");
        testMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setServoExtendedRange(Servo servo, int min, int max) {
        ServoControllerEx controller = (ServoControllerEx) servo.getController();
        int servoPort = servo.getPortNumber();
        PwmControl.PwmRange range = new PwmControl.PwmRange(min, max);
        controller.setServoPwmRange(servoPort, range);
    }

    public void loop(){

        telemetry.addData("motor1", testMotor1.getCurrentPosition());
        telemetry.addData("motor2", testMotor2.getCurrentPosition());

        telemetry.addData("servo", gamepad2.right_trigger);
        testServo1.setPosition(gamepad2.right_trigger);
        testServo2.setPosition(gamepad2.right_trigger);
        testServo3.setPosition(gamepad2.right_trigger);
        testServo4.setPosition(gamepad2.right_trigger);
    }

}
