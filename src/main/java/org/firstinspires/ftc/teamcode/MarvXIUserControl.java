package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="XIUC")
public class MarvXIUserControl extends OpMode {

    MarvXICommon marv;

    boolean rotateInStow = true;

    boolean hingeInHoldMode = false;
    boolean expandInHoldMode = false;
    boolean liftInHoldMode = false;

    static double LP_HORIZ_M = .30;
    static double LP_DIFF_M = .30;
    static double HP_HORIZ_M = .60;
    static double HP_DIFF_M = .60;

    static double D2_YAW_STICK_M = .20;
    static double D2_YAW_PAD_M = .20;

    static double D2_HINGE_STICK_M = 0.50;
    static double D2_HINGE_PAD_M = 1.00;

    public void init() {
        marv = new MarvXICommon(hardwareMap);
    }

    public void stop() {
        marv.stop();
    }

    public void loop() {

        marv.telemPoses(telemetry);
        telemetry.update();



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

        if (gamepad2.right_bumper || gamepad2.left_bumper) {
            rotateInStow = false;
        }

        if (gamepad2.back) {
            rotateInStow = true;
        }


        double g2ExpandInput = -gamepad2.right_stick_y;
        if (g2ExpandInput != 0) {
            if (expandInHoldMode) {
                marv.setExpandDefaultMode();
                expandInHoldMode = false;
            }
            if (g2ExpandInput < 0 && marv.expand.getCurrentPosition() <= 0) {marv.setExpandSpeed(0);} else {marv.setExpandSpeed(g2ExpandInput);}
        }
        else {
            /*if (!expandInHoldMode) {
                marv.setExpandHoldMode();
                expandInHoldMode = true;
            }
            if (marv.expand.getCurrentPosition() <= 0) { marv.setExpandSpeed(0);}*/
            marv.setExpandSpeed(0);
        }


        double g2HingeInput = gamepad2.left_stick_y * D2_HINGE_STICK_M;
        if (gamepad2.dpad_up) {
            g2HingeInput -= D2_HINGE_PAD_M;
        }
        else if (gamepad2.dpad_down) {
            g2HingeInput += D2_HINGE_PAD_M;
        }
        if (g2HingeInput != 0) {
            if (hingeInHoldMode) {
                marv.setHingeDefaultMode();
                hingeInHoldMode = false;
            }
            if (g2HingeInput < 0 && (marv.hingeL.getCurrentPosition() + marv.hingeR.getCurrentPosition()) / 2.0 <= 0) {marv.setHingeSpeed(0);} else {marv.setHingeSpeed(g2HingeInput);}
        }
        else {
            if (!hingeInHoldMode) {
                marv.setHingeHoldMode();
                hingeInHoldMode = true;
            }
            if ((marv.hingeL.getCurrentPosition() + marv.hingeR.getCurrentPosition()) / 2.0 <= 0) { marv.setHingeSpeed(0);}
            //marv.setHingeSpeed(0);
        }


        if (!rotateInStow) {
            if (gamepad2.right_trigger != 0 || gamepad2.left_trigger != 0) {
                marv.setIntakeSpeed(-Math.max((gamepad2.right_trigger + gamepad2.left_trigger) * 0.8, 0.8));
            } else if (marv.hingeIsInDrop() || gamepad2.a || gamepad2.b || gamepad2.x || gamepad2.y) {
                marv.setIntakeSpeed(0);
            } else {
                marv.setIntakeSpeed(0.8);
            }
        }
        else {
            marv.setIntakeSpeed(0);
        }


        double g1LiftInput;
        if (gamepad1.y) {
            g1LiftInput = 1;
        }
        else if (gamepad1.a) {
            g1LiftInput = -1;
        }
        else {
            g1LiftInput = 0;
        }
        if (g1LiftInput != 0) {
            if (liftInHoldMode) {
                marv.setLiftDefaultMode();
                liftInHoldMode = false;
            }
            if (g1LiftInput < 0 && marv.lift.getCurrentPosition() <= 0) {marv.setLiftSpeed(0);} else {marv.setLiftSpeed(g1LiftInput);}
        }
        else {
            if (!liftInHoldMode) {
                marv.setLiftHoldMode();
                liftInHoldMode = true;
            }
            //marv.setLiftSpeed(0);
        }


        double rot = gamepad2.left_stick_x * D2_YAW_STICK_M;
        if (gamepad2.dpad_right) {
            rot += D2_YAW_PAD_M;
        }
        else if (gamepad2.dpad_left) {
            rot -= D2_YAW_PAD_M;
        }


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
