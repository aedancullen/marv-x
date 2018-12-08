package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="UserTests2")
public class UserTests2 extends OpMode {

    MarvXCommon marv;

    static double LP_HORIZ_DIV = 1.5;
    static double LP_DIFF_DIV = 1.5;
    static double HP_HORIZ_DIV = 1;
    static double HP_DIFF_DIV = 1;

    DcMotor expandoHoriz;
    DcMotor expandoVertL;
    DcMotor expandoVertR;

    DcMotor horizSpin;
    Servo horizBoxL;
    Servo horizBoxR;

    Servo vertBoxL;
    Servo vertBoxR;
    Servo vertSpin;

    public void init(){
        expandoHoriz = hardwareMap.dcMotor.get("expandoHoriz");
        expandoHoriz.setDirection(DcMotorSimple.Direction.REVERSE);
        expandoVertL = hardwareMap.dcMotor.get("expandoVertL");
        expandoVertL.setDirection(DcMotorSimple.Direction.REVERSE);
        expandoVertR = hardwareMap.dcMotor.get("expandoVertR");

        horizSpin = hardwareMap.dcMotor.get("horizSpin");

        // ONE REVERSE
        horizBoxL = hardwareMap.servo.get("horizBoxL");
        setServoExtendedRange(horizBoxL, 500, 2500);
        horizBoxR = hardwareMap.servo.get("horizBoxR");
        horizBoxR.setDirection(Servo.Direction.REVERSE);
        setServoExtendedRange(horizBoxR, 500, 2500);

        // ONE REVERSE
        vertBoxL = hardwareMap.servo.get("vertBoxL");
        setServoExtendedRange(vertBoxL, 500, 2500);
        vertBoxR = hardwareMap.servo.get("vertBoxR");
        vertBoxR.setDirection(Servo.Direction.REVERSE);
        setServoExtendedRange(vertBoxR, 500, 2500);

        vertSpin = hardwareMap.servo.get("vertSpin");
        setServoExtendedRange(vertSpin, 500, 2500);
    }

    public void setServoExtendedRange(Servo servo, int min, int max) {
        ServoControllerEx controller = (ServoControllerEx) servo.getController();
        int servoPort = servo.getPortNumber();
        PwmControl.PwmRange range = new PwmControl.PwmRange(min, max);
        controller.setServoPwmRange(servoPort, range);
    }

    public void loop(){

        /*if (gamepad1.right_bumper) {
            marv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else {
            marv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if (gamepad1.left_bumper) {
            double horiz;
            horiz = (gamepad1.right_trigger / HP_HORIZ_DIV) - (gamepad1.left_trigger / HP_HORIZ_DIV);
            marv.drive(-gamepad1.left_stick_y / HP_DIFF_DIV, -gamepad1.right_stick_y / HP_DIFF_DIV, horiz);
        }
        else {
            double horiz;
            horiz = (gamepad1.right_trigger / LP_HORIZ_DIV) - (gamepad1.left_trigger / LP_HORIZ_DIV);
            marv.drive(-gamepad1.left_stick_y / LP_DIFF_DIV, -gamepad1.right_stick_y / LP_DIFF_DIV, horiz);
        }*/

        telemetry.addData("expandoHoriz", expandoHoriz.getCurrentPosition());
        telemetry.addData("expandoVertL", expandoVertL.getCurrentPosition());
        telemetry.addData("expandoVertR", expandoVertR.getCurrentPosition());

        telemetry.addData("horizSpin", horizSpin.getCurrentPosition());

        telemetry.addData("servo", gamepad2.right_trigger);

        telemetry.update();


        vertSpin.setPosition(gamepad2.right_trigger);
        vertBoxL.setPosition(gamepad2.right_trigger);
        vertBoxR.setPosition(gamepad2.right_trigger);
        horizBoxL.setPosition(gamepad2.right_trigger);
        horizBoxR.setPosition(gamepad2.right_trigger);

        if (gamepad2.a) {
            expandoVertL.setPower(0.2);
            expandoVertR.setPower(0.2);
        }
        else if (gamepad2.b) {
            expandoVertL.setPower(-0.2);
            expandoVertR.setPower(-0.2);
        }
        else {
            expandoVertL.setPower(0);
            expandoVertR.setPower(0);
        }


    }

}
