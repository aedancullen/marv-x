package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="UserControl 3")
public class MarvXUserControlV3 extends OpMode {

    static double LP_HORIZ_M = .33;
    static double LP_DIFF_M = .33;
    static double HP_HORIZ_M = .75;
    static double HP_DIFF_M = .75;

    long dumpTimer;
    long waitTimer;
    long upTimer;

    boolean liftmode = false;
    boolean horizLiftIsDisable = false;

    boolean transferPrimed;
    boolean transferGo = false;

    MarvXCommonV3 marv;

    public void init() {
        marv = new MarvXCommonV3(hardwareMap, false);
    }

    public void start() {
        marv.horizLiftL.setPosition(MarvConstantsV3.HORIZ_LIFT_UP_NEUTRAL);
        marv.horizLiftR.setPosition(MarvConstantsV3.HORIZ_LIFT_UP_NEUTRAL);
        marv.drop.setPosition(MarvConstantsV3.DROP_ANGLE_FLAT);
        marv.swop.setPosition(MarvConstantsV3.SWOP_ANGLE_NORMAL);
    }

    public void loop() {
        double horiz;
        horiz = (gamepad1.right_trigger * HP_HORIZ_M) - (gamepad1.left_trigger * HP_HORIZ_M);
        marv.drive(-gamepad1.left_stick_y * HP_DIFF_M, -gamepad1.right_stick_y * HP_DIFF_M, horiz);

        if (gamepad2.a) {
            marv.dropTarget = MarvXCommonV3.DropTarget.NEAR;
        }
        else if (gamepad2.b || gamepad2.x) {
            marv.dropTarget = MarvXCommonV3.DropTarget.MID;
        }
        else if (gamepad2.y) {
            marv.dropTarget = MarvXCommonV3.DropTarget.FAR;
        }


        if (!liftmode) {
            marv.runAutomation((transferGo) || gamepad2.back, gamepad1.right_bumper || gamepad2.back);
            if ((transferGo && gamepad2.a) || gamepad2.back) {
                transferGo = false;
            }
        }


        if (gamepad2.left_trigger > 0.5 && !horizLiftIsDisable) {
            marv.horizLiftL.setPosition(MarvConstantsV3.HORIZ_LIFT_DOWN);
            marv.horizLiftR.setPosition(MarvConstantsV3.HORIZ_LIFT_DOWN);
            marv.horizLiftL.getController().pwmDisable();
            horizLiftIsDisable = true;
        }
        else if (gamepad2.left_bumper && horizLiftIsDisable) {
            marv.horizLiftL.getController().pwmEnable();
            marv.horizLiftL.setPosition(MarvConstantsV3.HORIZ_LIFT_UP_NEUTRAL);
            marv.horizLiftR.setPosition(MarvConstantsV3.HORIZ_LIFT_UP_NEUTRAL);
            horizLiftIsDisable = false;
        }


        if (gamepad2.dpad_down || gamepad2.dpad_up) {
            marv.horizSpinL.setPower(0);
            marv.horizSpinR.setPower(0);
        }
        else if (gamepad2.dpad_right) {
            marv.horizSpinL.setPower(-MarvConstantsV3.UC_HORIZSPIN_EJECT);
            marv.horizSpinR.setPower(0);
        }
        else if (gamepad2.dpad_left) {
            marv.horizSpinR.setPower(-MarvConstantsV3.UC_HORIZSPIN_EJECT);
            marv.horizSpinL.setPower(0);
        }
        else {
            if (!liftmode) {
                if (Math.abs(marv.horizLiftL.getPosition() - MarvConstantsV3.HORIZ_LIFT_DOWN) < 0.01) {
                    marv.horizSpinL.setPower(MarvConstantsV3.UC_HORIZSPIN_INTAKE);
                    marv.horizSpinR.setPower(MarvConstantsV3.UC_HORIZSPIN_INTAKE);
                    dumpTimer = System.currentTimeMillis();
                    waitTimer = dumpTimer;
                    upTimer = dumpTimer;
                }
                else if (Math.abs(marv.horizLiftL.getPosition() - MarvConstantsV3.HORIZ_LIFT_UP_DUMPING) < 0.01) {
                    if (System.currentTimeMillis() - upTimer < MarvConstantsV3.UC_HORIZLIFT_TOUP_MS) {
                        waitTimer = System.currentTimeMillis();
                    }
                    if (System.currentTimeMillis() - waitTimer < MarvConstantsV3.UC_HORIZLIFT_TOWAIT_MS) {
                        dumpTimer = System.currentTimeMillis();
                    }
                    if (System.currentTimeMillis() - dumpTimer > MarvConstantsV3.UC_HORIZLIFT_TODUMP_MS && marv.automationState == MarvXCommonV3.AutomationState.READY) {
                        marv.horizSpinL.setPower(-MarvConstantsV3.UC_HORIZSPIN_TRANSFER);
                        marv.horizSpinR.setPower(-MarvConstantsV3.UC_HORIZSPIN_TRANSFER);
                        transferPrimed = true;
                    }
                }
                else if (Math.abs(marv.horizLiftL.getPosition() - MarvConstantsV3.HORIZ_LIFT_UP_WAITING) < 0.01) {
                    marv.horizSpinL.setPower(MarvConstantsV3.UC_HORIZSPIN_HOLD);
                    marv.horizSpinR.setPower(MarvConstantsV3.UC_HORIZSPIN_HOLD);
                    dumpTimer = System.currentTimeMillis();
                    if (System.currentTimeMillis() - upTimer < MarvConstantsV3.UC_HORIZLIFT_TOUP_MS) {
                        waitTimer = dumpTimer;
                    }
                }
                else { // NEUTRAL
                    marv.horizSpinL.setPower(MarvConstantsV3.UC_HORIZSPIN_HOLD);
                    marv.horizSpinR.setPower(MarvConstantsV3.UC_HORIZSPIN_HOLD);
                    dumpTimer = System.currentTimeMillis();
                    waitTimer = dumpTimer;
                }
            }
            else {
                marv.horizSpinL.setPower(0);
                marv.horizSpinR.setPower(0);
            }
        }




        if (gamepad2.right_trigger > 0 && !gamepad2.right_bumper
                && marv.automationState != MarvXCommonV3.AutomationState.UP
                && marv.automationState != MarvXCommonV3.AutomationState.DROP) {

            if ((marv.expandoHorizL.getCurrentPosition() + marv.expandoHorizR.getCurrentPosition()) / 2.0 < MarvConstantsV3.EXPANDO_HORIZ_UP) {
                marv.expandoHorizL.setPower(gamepad2.right_trigger * 1);
                marv.expandoHorizR.setPower(gamepad2.right_trigger * 1);
            }
            else {
                marv.expandoHorizL.setPower(0);
                marv.expandoHorizR.setPower(0);
            }

            if (Math.abs(marv.horizLiftL.getPosition() - MarvConstantsV3.HORIZ_LIFT_UP_DUMPING) < 0.01) {
                marv.horizLiftL.setPosition(MarvConstantsV3.HORIZ_LIFT_UP_NEUTRAL);
                marv.horizLiftR.setPosition(MarvConstantsV3.HORIZ_LIFT_UP_NEUTRAL);
            }
        }
        else if (gamepad2.right_trigger == 0 && gamepad2.right_bumper) {
            if ((marv.expandoHorizL.getCurrentPosition() + marv.expandoHorizR.getCurrentPosition()) / 2.0 > MarvConstantsV3.EXPANDO_HORIZ_SAFE + MarvConstantsV3.UC_EXPANDOHORIZ_BUF) {
                marv.expandoHorizL.setPower(-1);
                marv.expandoHorizR.setPower(-1);
                if (Math.abs(marv.horizLiftL.getPosition() - MarvConstantsV3.HORIZ_LIFT_UP_NEUTRAL) < 0.01) {
                    marv.horizLiftL.setPosition(MarvConstantsV3.HORIZ_LIFT_UP_WAITING);
                    marv.horizLiftR.setPosition(MarvConstantsV3.HORIZ_LIFT_UP_WAITING);
                }
            }
            else {
                if ((marv.expandoHorizL.getCurrentPosition() + marv.expandoHorizR.getCurrentPosition()) / 2.0 < MarvConstantsV3.EXPANDO_HORIZ_SAFE - MarvConstantsV3.UC_EXPANDOHORIZ_BUF) {
                    marv.expandoHorizL.setPower(0.25);
                    marv.expandoHorizR.setPower(0.25);
                }
                else {
                    marv.expandoHorizL.setPower(0);
                    marv.expandoHorizR.setPower(0);
                }
                if (Math.abs(marv.horizLiftL.getPosition() - MarvConstantsV3.HORIZ_LIFT_UP_WAITING) < 0.01) {
                    marv.horizLiftL.setPosition(MarvConstantsV3.HORIZ_LIFT_UP_DUMPING);
                    marv.horizLiftR.setPosition(MarvConstantsV3.HORIZ_LIFT_UP_DUMPING);
                }
            }

        }
        else {
            marv.expandoHorizL.setPower(0);
            marv.expandoHorizR.setPower(0);
            if (Math.abs(marv.horizLiftL.getPosition() - MarvConstantsV3.HORIZ_LIFT_UP_DUMPING) < 0.01 || Math.abs(marv.horizLiftL.getPosition() - MarvConstantsV3.HORIZ_LIFT_UP_WAITING) < 0.01) {
                marv.horizLiftL.setPosition(MarvConstantsV3.HORIZ_LIFT_UP_NEUTRAL);
                marv.horizLiftR.setPosition(MarvConstantsV3.HORIZ_LIFT_UP_NEUTRAL);
            }
            if (transferPrimed) {
                transferPrimed = false;
                transferGo = true;
            }
        }
    }


}
