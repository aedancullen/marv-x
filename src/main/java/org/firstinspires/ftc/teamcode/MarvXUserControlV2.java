package org.firstinspires.ftc.teamcode;;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.MarvXCommonV2;

@TeleOp(name="UserControlV2")
public class MarvXUserControlV2 extends OpMode {

    static double LP_HORIZ_M = .30;
    static double LP_DIFF_M = .30;
    static double HP_HORIZ_M = .60;
    static double HP_DIFF_M = .60;

    MarvXCommonV2 marv;

    public void init() {
        marv = new MarvXCommonV2(hardwareMap, true);
        marv.horizLiftL.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);
        marv.horizLiftR.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);
    }

    public void loop() {

        if (gamepad1.right_bumper) {
            marv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else {
            marv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if (!gamepad1.left_bumper) {
            double horiz;
            horiz = (gamepad1.right_trigger * HP_HORIZ_M) - (gamepad1.left_trigger * HP_HORIZ_M);
            marv.drive(gamepad1.right_stick_y * HP_DIFF_M, gamepad1.left_stick_y * HP_DIFF_M, -horiz);
        }
        else {
            double horiz;
            horiz = (gamepad1.right_trigger * LP_HORIZ_M) - (gamepad1.left_trigger * LP_HORIZ_M);
            marv.drive(gamepad1.right_stick_y * LP_DIFF_M, gamepad1.left_stick_y * LP_DIFF_M, -horiz);
        }



        marv.runAutomation(gamepad2.a, gamepad2.left_stick_button && gamepad2.right_stick_button);

        if (gamepad2.left_trigger > 0.5 && marv.horizLiftL.getPosition() == MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL) {
            marv.horizLiftL.setPosition(MarvConstantsV2.HORIZ_LIFT_DOWN);
            marv.horizLiftR.setPosition(MarvConstantsV2.HORIZ_LIFT_DOWN);
        }
        if (gamepad2.left_bumper && marv.horizLiftL.getPosition() == MarvConstantsV2.HORIZ_LIFT_DOWN) {
            marv.horizLiftL.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);
            marv.horizLiftR.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);
        }


        if (gamepad2.dpad_down || gamepad2.dpad_up) {
            marv.horizSpinL.setPower(-1);
            marv.horizSpinR.setPower(-1);
        }
        else if (gamepad2.dpad_right || gamepad2.dpad_left) {
            marv.horizSpinL.setPower(0);
            marv.horizSpinR.setPower(0);
        }
        else {
            marv.horizSpinL.setPower(1);
            marv.horizSpinR.setPower(1);
        }




        if (gamepad2.right_trigger > 0 && !gamepad2.right_bumper) {
            if (marv.expandoHorizL.getCurrentPosition() < MarvConstantsV2.EXPANDO_HORIZ_UP) {
                marv.expandoHorizL.setPower(gamepad2.right_trigger * 0.5);
            }
            else {
                marv.expandoHorizL.setPower(0);
            }

            if (marv.horizLiftL.getPosition() == MarvConstantsV2.HORIZ_LIFT_UP_DUMPING) {
                marv.horizLiftL.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);
                marv.horizLiftR.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);
            }
        }
        else if (gamepad2.right_trigger == 0 && gamepad2.right_bumper) {
            if (marv.expandoHorizL.getCurrentPosition() > MarvConstantsV2.EXPANDO_HORIZ_SAFE) {
                marv.expandoHorizL.setPower(-0.5);

                if (marv.horizLiftL.getPosition() == MarvConstantsV2.HORIZ_LIFT_UP_DUMPING) {
                    marv.horizLiftL.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);
                    marv.horizLiftR.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);
                }
            }
            else {
                marv.expandoHorizL.setPower(0);
                marv.horizLiftL.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_DUMPING);
                marv.horizLiftR.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_DUMPING);
            }
        }
        else {
            marv.expandoHorizL.setPower(0);

            if (marv.horizLiftL.getPosition() == MarvConstantsV2.HORIZ_LIFT_UP_DUMPING) {
                marv.horizLiftL.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);
                marv.horizLiftR.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);
            }
        }
    }
}

