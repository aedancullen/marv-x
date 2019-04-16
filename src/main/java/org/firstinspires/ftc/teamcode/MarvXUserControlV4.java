package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="UserControl")
public class MarvXUserControlV4 extends OpMode {

    static double LP_HORIZ_M = .33;
    static double LP_DIFF_M = .33;
    static double HP_HORIZ_M = .75;
    static double HP_DIFF_M = .75;

    MarvXCommonV3 marv;

    public void init() {
        marv = new MarvXCommonV3(hardwareMap, false);
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

        double controlSlide = 0;
        if (gamepad2.right_trigger > 0 && !gamepad2.right_bumper) {
            controlSlide = gamepad2.right_trigger;
        }
        else if (gamepad2.right_bumper && gamepad2.right_trigger == 0) {
            controlSlide = -1;
        }

        boolean controlUp;
        boolean controlDown;
        if (gamepad2.left_bumper && gamepad2.left_trigger == 0) {
            controlUp = true;
            controlDown = false;
        }
        else if (gamepad2.left_trigger > 0.5 && !gamepad2.left_bumper) {
            controlUp = false;
            controlDown = true;
        }
        else {
            controlUp = false;
            controlDown = false;
        }

        marv.runIntakeAutomation(controlSlide, controlUp, controlDown, gamepad2.a);
        if (marv.transferDone) {
            transferPrimed = true;
        }

        boolean transferGo;
        if (transferPrimed && !gamepad2.back) {
            transferGo = true;
            transferPrimed = false;
        }
        else {
            transferGo = false;
        }
        marv.runAutomation(marv.transferGo, !gamepad2.back);

    }


}
