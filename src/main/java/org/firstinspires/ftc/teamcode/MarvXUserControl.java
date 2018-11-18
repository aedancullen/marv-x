package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Marv Mk10 UserTests")
public class MarvXUserControl extends OpMode {

    MarvXCommon marv;

    static double LP_HORIZ_DIV = 1.5;
    static double LP_DIFF_DIV = 1.5;
    static double HP_HORIZ_DIV = 1;
    static double HP_DIFF_DIV = 1;

    public void init(){
        marv = new MarvXCommon(hardwareMap);
    }

    public void init_loop() {
        telemetry.addData("qpX", marv.getQuadPacerX());
        telemetry.addData("qpY", marv.getQuadPacerY());
        Orientation angles = marv.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        telemetry.addData("first", angles.firstAngle);
        telemetry.addData("second", angles.secondAngle);
        telemetry.addData("third", angles.thirdAngle);
        telemetry.update();
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

        telemetry.addData("qpX", marv.getQuadPacerX());
        telemetry.addData("qpY", marv.getQuadPacerY());

    }

}
