package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.MarvXCommonV2;

@TeleOp(name="UserControlV2")
public class MarvXUserControlV2 extends OpMode {

    static double LP_HORIZ_M = .33;
    static double LP_DIFF_M = .33;
    static double HP_HORIZ_M = .75;
    static double HP_DIFF_M = .75;

    MarvXCommonV2 marv;

    boolean liftmode = false;
    boolean horizLiftIsDisable = false;

    public void init() {
        marv = new MarvXCommonV2(hardwareMap, true);
        //marv.horizLiftL.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);
        //marv.horizLiftR.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);

        marv.vertLatch.setPosition(MarvConstantsV2.VERT_LATCH_OPEN);
        //marv.vertSwing.setPosition(MarvConstantsV2.VERT_SWING_CENTER);
    }

    long dumpTimer;

    public void loop() {

        telemetry.addData("expando", marv.expandoHorizL.getCurrentPosition());
        telemetry.update();

        if (gamepad1.right_bumper) {
            marv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        else {
            marv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (gamepad1.left_bumper) {
            double horiz;
            horiz = (gamepad1.right_trigger * LP_HORIZ_M) - (gamepad1.left_trigger * LP_HORIZ_M);
            marv.drive(-gamepad1.left_stick_y * LP_DIFF_M, -gamepad1.right_stick_y * LP_DIFF_M, horiz);
        }
        else {
            double horiz;
            horiz = (gamepad1.right_trigger * HP_HORIZ_M) - (gamepad1.left_trigger * HP_HORIZ_M);
            marv.drive(-gamepad1.left_stick_y * HP_DIFF_M, -gamepad1.right_stick_y * HP_DIFF_M, horiz);
        }




        if (gamepad2.back && (gamepad2.x || gamepad2.y)) {
            liftmode = true;
        }


        if (!liftmode) {
            if (!gamepad1.back) {marv.runAutomation(gamepad2.a, gamepad2.left_stick_button && gamepad2.right_stick_button);}
        }
        else {
            if (gamepad2.y) {
                marv.expandoVertL.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_SAFE);
                marv.expandoVertR.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_SAFE);
                marv.expandoVertL.setPower(MarvConstantsV2.EXPANDO_VERT_TOSAFE_SPEED);
                marv.expandoVertR.setPower(MarvConstantsV2.EXPANDO_VERT_TOSAFE_SPEED);
            }
            else if (gamepad2.x) {
                marv.expandoVertL.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_DOWN);
                marv.expandoVertR.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_DOWN);
                marv.expandoVertL.setPower(MarvConstantsV2.EXPANDO_VERT_TODOWN_SPEED);
                marv.expandoVertR.setPower(MarvConstantsV2.EXPANDO_VERT_TODOWN_SPEED);
            }
        }



        if (gamepad2.left_trigger > 0.5/* && !horizLiftIsDisable) && Math.abs(marv.horizLiftL.getPosition() - MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL) < 0.01*/) {
            marv.horizLiftL.setPosition(MarvConstantsV2.HORIZ_LIFT_DOWN);
            marv.horizLiftR.setPosition(MarvConstantsV2.HORIZ_LIFT_DOWN);
            //marv.horizLiftL.getController().pwmDisable();
            horizLiftIsDisable = true;
        }
        else if (gamepad2.left_bumper/* && horizLiftIsDisable*/) {
            //marv.horizLiftL.getController().pwmEnable();
            marv.horizLiftL.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);
            marv.horizLiftR.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);
            horizLiftIsDisable = false;
        }


        if (gamepad2.dpad_down || gamepad2.dpad_up) {
            marv.horizSpinL.setPower(0);
            marv.horizSpinR.setPower(0);
        }
        else if (gamepad2.dpad_right) {
            marv.horizSpinL.setPower(-0.75);
        }
        else if (gamepad2.dpad_left) {
            marv.horizSpinR.setPower(-0.75);
        }
        else {
            if (!liftmode) {
                if (Math.abs(marv.horizLiftL.getPosition() - MarvConstantsV2.HORIZ_LIFT_DOWN) < 0.01) {
                    marv.horizSpinL.setPower(0.75);
                    marv.horizSpinR.setPower(0.75);
                    dumpTimer = System.currentTimeMillis();
                }
                else if (Math.abs(marv.horizLiftL.getPosition() - MarvConstantsV2.HORIZ_LIFT_UP_DUMPING) < 0.01) {
                    if (System.currentTimeMillis() - dumpTimer > 250) {
                        marv.horizSpinL.setPower(-0.45);
                        marv.horizSpinR.setPower(-0.45);
                    }
                }
                else {
                    marv.horizSpinL.setPower(/*0.15*/0);
                    marv.horizSpinR.setPower(/*0.15*/0);
                    dumpTimer = System.currentTimeMillis();
                }
            }
            else {
                marv.horizSpinL.setPower(0);
                marv.horizSpinR.setPower(0);
            }
        }




        if (gamepad2.right_trigger > 0 && !gamepad2.right_bumper) {
            if (marv.expandoHorizL.getCurrentPosition() < MarvConstantsV2.EXPANDO_HORIZ_UP) {
                marv.expandoHorizL.setPower(gamepad2.right_trigger * 1);
            }
            else {
                marv.expandoHorizL.setPower(0);
            }

            if (Math.abs(marv.horizLiftL.getPosition() - MarvConstantsV2.HORIZ_LIFT_UP_DUMPING) < 0.01) {
                marv.horizLiftL.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);
                marv.horizLiftR.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);
            }
        }
        else if (gamepad2.right_trigger == 0 && gamepad2.right_bumper) {
            if (marv.expandoHorizL.getCurrentPosition() > MarvConstantsV2.EXPANDO_HORIZ_SAFE) {
                marv.expandoHorizL.setPower(-1);
                if (Math.abs(marv.horizLiftL.getPosition() - MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL) < 0.01) {
                    marv.horizLiftL.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_WAITING);
                    marv.horizLiftR.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_WAITING);
                }
            }
            else {
                marv.expandoHorizL.setPower(0);
                if (Math.abs(marv.horizLiftL.getPosition() - MarvConstantsV2.HORIZ_LIFT_UP_WAITING) < 0.01) {
                    marv.horizLiftL.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_DUMPING);
                    marv.horizLiftR.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_DUMPING);
                }
            }

        }
        else {
            marv.expandoHorizL.setPower(0);

            if (Math.abs(marv.horizLiftL.getPosition() - MarvConstantsV2.HORIZ_LIFT_UP_DUMPING) < 0.01 || Math.abs(marv.horizLiftL.getPosition() - MarvConstantsV2.HORIZ_LIFT_UP_WAITING) < 0.01) {
                marv.horizLiftL.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);
                marv.horizLiftR.setPosition(MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL);
            }
        }
    }
}

