package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MarvXCommon;

@TeleOp(name="UserControl")
public class MarvXUserControl extends OpMode {

    static double LP_HORIZ_DIV = 1.5;
    static double LP_DIFF_DIV = 1.5;
    static double HP_HORIZ_DIV = 1;
    static double HP_DIFF_DIV = 1;

    MarvXCommon marv;

    public void init() {
        marv = new MarvXCommon(hardwareMap, true);
        telemetry.addData("IntakeState", marv.intakeState);
        telemetry.addData("ExpandoHorizState", marv.expandoHorizState);
        telemetry.addData("ExpandoVertState", marv.expandoVertState);
        telemetry.addData("BoxLiftState", marv.boxLiftState);
        telemetry.addData("BoxSpinState", marv.boxSpinState);
        telemetry.addData("AutomationState", marv.automationState);
    }

    public void loop() {
        telemetry.update();

        if (gamepad1.right_bumper) {
            marv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else {
            marv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if (gamepad1.left_bumper) {
            double horiz;
            horiz = (gamepad1.right_trigger / HP_HORIZ_DIV) - (gamepad1.left_trigger / HP_HORIZ_DIV);
            marv.drive(gamepad1.right_stick_y / HP_DIFF_DIV, gamepad1.left_stick_y / HP_DIFF_DIV, -horiz);
        }
        else {
            double horiz;
            horiz = (gamepad1.right_trigger / LP_HORIZ_DIV) - (gamepad1.left_trigger / LP_HORIZ_DIV);
            marv.drive(gamepad1.right_stick_y / LP_DIFF_DIV, gamepad1.left_stick_y / LP_DIFF_DIV, -horiz);
        }

        double intakeManualPower;
        double expandoHorizManualPower;

        if (gamepad2.dpad_up || gamepad2.dpad_down) {
            intakeManualPower = -1;
        }
        else if (gamepad2.dpad_right || gamepad2.dpad_left) {
            intakeManualPower = -1;
        }
        else {
            intakeManualPower = 1;
        }


        if (gamepad2.right_bumper) {
            expandoHorizManualPower = -0.5;
        }
        else {
            expandoHorizManualPower = gamepad2.right_trigger * 0.5;
        }


        if (marv.expandoHorizState == MarvXCommon.ExpandoHorizState.MANUAL) {
            if (gamepad2.left_trigger > 0.5 && marv.intakeState != MarvXCommon.IntakeState.STROBE_DOWN && marv.intakeState != MarvXCommon.IntakeState.DOWN_NEUTRAL) {
                marv.intakeState = MarvXCommon.IntakeState.STROBE_DOWN;
            }
            else if (gamepad2.left_bumper && marv.intakeState != MarvXCommon.IntakeState.STROBE_UP && marv.intakeState != MarvXCommon.IntakeState.UP_NEUTRAL) {
                marv.intakeState = MarvXCommon.IntakeState.STROBE_UP;
            }
        }

        marv.runAutomationStateMachine(gamepad2.a, gamepad2.b, (float)intakeManualPower, (float)expandoHorizManualPower);
    }
}

