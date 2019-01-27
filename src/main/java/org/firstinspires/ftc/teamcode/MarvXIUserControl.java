package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MarvXIUserControl extends OpMode {

    MarvXICommon marv;

    boolean rotateInStow = true;

    boolean liftInHoldMode = false;
    boolean expandInHoldMode = false;

    static double LP_HORIZ_M = .30;
    static double LP_DIFF_M = .30;
    static double HP_HORIZ_M = .60;
    static double HP_DIFF_M = .60;

    static double D2_YAW_M = .20;

    public void init() {
        marv = new MarvXICommon(hardwareMap);
    }

    public void loop() {
        if (rotateInStow) {
            marv.setRotateIn();
        }
        else {
            if (gamepad2.right_bumper) {
                marv.setRotateR();
            }
            else if (gamepad2.left_bumper) {
                marv.setRotateL();
            }
            else {
                marv.setRotateStraight();
            }
        }

        if (gamepad2.back) {
            rotateInStow = true;
        }


        double g2ExpandInput = gamepad2.right_trigger - gamepad2.left_trigger;
        if (g2ExpandInput != 0) {
            if (expandInHoldMode) {
                marv.setExpandDefaultMode();
                expandInHoldMode = false;
            }
            marv.setExpandSpeed(g2ExpandInput);
        }
        else {
            if (!expandInHoldMode) {
                marv.setExpandHoldMode();
                expandInHoldMode = true;
            }
        }


        double g2HingeInput;



        double rot = gamepad2.left_stick_x * D2_YAW_M;

        if (gamepad1.right_bumper) {
            marv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else {
            marv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if (gamepad1.left_bumper) {
            double horiz;
            horiz = (gamepad1.right_trigger * LP_HORIZ_M) - (gamepad1.left_trigger * LP_HORIZ_M);
            marv.drive(gamepad1.right_stick_y * LP_DIFF_M, gamepad1.left_stick_y * LP_DIFF_M, -horiz, rot);
        }
        else {
            double horiz;
            horiz = (gamepad1.right_trigger * HP_HORIZ_M) - (gamepad1.left_trigger * HP_HORIZ_M);
            marv.drive(gamepad1.right_stick_y * HP_DIFF_M, gamepad1.left_stick_y * HP_DIFF_M, -horiz, rot);
        }

    }
}
