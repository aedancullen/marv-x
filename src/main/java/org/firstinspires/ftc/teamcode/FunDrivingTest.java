package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.firstinspires.ftc.teamcode.MarvXUserControlV2.HP_DIFF_M;
import static org.firstinspires.ftc.teamcode.MarvXUserControlV2.HP_HORIZ_M;
import static org.firstinspires.ftc.teamcode.MarvXUserControlV2.LP_DIFF_M;
import static org.firstinspires.ftc.teamcode.MarvXUserControlV2.LP_HORIZ_M;

@TeleOp(name="FunDrivingTest")
public class FunDrivingTest extends OpMode {

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    DcMotor.ZeroPowerBehavior lastZeroPowerBehavior;

    public void init() {
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

    }





    public void loop() {

        if (gamepad1.right_bumper) {
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else {
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if (!gamepad1.left_bumper) {
            double horiz;
            horiz = (gamepad1.right_trigger * HP_HORIZ_M) - (gamepad1.left_trigger * HP_HORIZ_M);
            drive(gamepad1.right_stick_y * HP_DIFF_M, gamepad1.left_stick_y * HP_DIFF_M, -horiz);
        }
        else {
            double horiz;
            horiz = (gamepad1.right_trigger * LP_HORIZ_M) - (gamepad1.left_trigger * LP_HORIZ_M);
            drive(gamepad1.right_stick_y * LP_DIFF_M, gamepad1.left_stick_y * LP_DIFF_M, -horiz);
        }

    }













    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        if (behavior != lastZeroPowerBehavior) {
            lastZeroPowerBehavior = behavior;
            fl.setZeroPowerBehavior(behavior);
            fr.setZeroPowerBehavior(behavior);
            fr.setDirection(DcMotorSimple.Direction.REVERSE);
            bl.setZeroPowerBehavior(behavior);
            br.setZeroPowerBehavior(behavior);
            br.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }


    public void drive(
            double vertL,
            double vertR,
            double horiz
    ) {
        double rot = 0;

        double flp = vertL + rot + horiz;
        fl.setPower(Math.max(Math.min(flp, 1), -1));
        double frp = vertR - rot - horiz;
        fr.setPower(Math.max(Math.min(frp, 1), -1));
        double blp = vertL + rot - horiz;
        bl.setPower(Math.max(Math.min(blp, 1), -1));
        double brp = vertR - rot + horiz;
        br.setPower(Math.max(Math.min(brp, 1), -1));
    }



}
