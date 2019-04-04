package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

public class MarvXCommonV3 {

    BNO055IMU imu;

    DcMotor fr;
    DcMotor br;
    DcMotor bl;
    DcMotor fl;

    DcMotor expandoHorizL;
    DcMotor expandoHorizR;

    DcMotor expandoVert;
    DcMotor expandoDiag;

    Servo horizLiftL;
    Servo horizLiftR;

    CRServo horizSpinL;
    CRServo horizSpinR;

    Servo drop;
    Servo swop;

    DcMotor.ZeroPowerBehavior lastZeroPowerBehavior;

    enum AutomationState{STALIN, READY, UP, DROP, UNDROP, DOWN}
    public enum DropTarget{FAR, NEAR, MID}

    public DropTarget dropTarget = DropTarget.NEAR;

    AutomationState automationState;
    AutomationState lastAutomationState;

    public DcMotor getQuadPacerMotorX() {
        return br;
    }
    public DcMotor getQuadPacerMotorY() {
        return bl;
    }

    public void drive(
            double vertL,
            double vertR,
            double horiz
    ) {
        drive(vertL, vertR, horiz, 0);
    }

    public void drive(
            double vertL,
            double vertR,
            double horiz,
            double rot
    ) {
        double flp = vertL + rot + horiz;
        fl.setPower(Math.max(Math.min(flp, 1), -1));
        double frp = vertR - rot - horiz;
        fr.setPower(Math.max(Math.min(frp, 1), -1));
        double blp = vertL + rot - horiz;
        bl.setPower(Math.max(Math.min(blp, 1), -1));
        double brp = vertR - rot + horiz;
        br.setPower(Math.max(Math.min(brp, 1), -1));
    }


    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        if (behavior != lastZeroPowerBehavior) {
            lastZeroPowerBehavior = behavior;
            fl.setZeroPowerBehavior(behavior);
            fr.setZeroPowerBehavior(behavior);
            bl.setZeroPowerBehavior(behavior);
            br.setZeroPowerBehavior(behavior);
        }
    }

    public void setEncoderBehavior(DcMotor.RunMode behavior) {
        fl.setMode(behavior);
        fr.setMode(behavior);
        bl.setMode(behavior);
        br.setMode(behavior);
    }

    public MarvXCommonV3(HardwareMap hardwareMap, boolean isAuto) {
        fr = hardwareMap.dcMotor.get("fr");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);

        br = hardwareMap.dcMotor.get("br");
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        bl = hardwareMap.dcMotor.get("bl");

        fl = hardwareMap.dcMotor.get("fl");

        setEncoderBehavior(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderBehavior(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        expandoHorizL = hardwareMap.dcMotor.get("expandoHorizL");
        expandoHorizL.setDirection(DcMotorSimple.Direction.REVERSE);
        expandoHorizL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (isAuto) {expandoHorizL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
        expandoHorizL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        expandoHorizL.setPower(0);

        expandoHorizR = hardwareMap.dcMotor.get("expandoHorizR");
        expandoHorizR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (isAuto) {expandoHorizR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
        expandoHorizR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        expandoHorizR.setPower(0);

        expandoVert = hardwareMap.dcMotor.get("expandoVert");
        expandoVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (isAuto) {expandoVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
        expandoVert.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        expandoVert.setPower(0);

        expandoDiag = hardwareMap.dcMotor.get("expandoDiag");
        expandoDiag.setDirection(DcMotorSimple.Direction.REVERSE);
        expandoDiag.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (isAuto) {expandoDiag.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
        expandoDiag.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        expandoDiag.setPower(0);


        horizLiftL = hardwareMap.servo.get("horizLiftL");
        setServoExtendedRange(horizLiftL, 500, 2500);
        horizLiftR = hardwareMap.servo.get("horizLiftR");
        horizLiftR.setDirection(Servo.Direction.REVERSE);
        setServoExtendedRange(horizLiftR, 500, 2500);

        horizSpinL = hardwareMap.crservo.get("horizSpinL");
        horizSpinR = hardwareMap.crservo.get("horizSpinR");
        horizSpinR.setDirection(DcMotorSimple.Direction.REVERSE);

        drop = hardwareMap.servo.get("drop");
        setServoExtendedRange(drop, 500, 2500);

        swop = hardwareMap.servo.get("swop");
        setServoExtendedRange(swop, 500, 2500);

    }

    public void setServoExtendedRange(Servo servo, int min, int max) {
        ServoControllerEx controller = (ServoControllerEx) servo.getController();
        int servoPort = servo.getPortNumber();
        PwmControl.PwmRange range = new PwmControl.PwmRange(min, max);
        controller.setServoPwmRange(servoPort, range);
    }

    public void runAutomation(boolean goStart, boolean goDrop) {

        if (automationState == AutomationState.STALIN && lastAutomationState != AutomationState.STALIN) {
            expandoDiag.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            expandoDiag.setPower(MarvConstantsV3.EXPANDO_DIAG_STALIN_POWER);
            stalinTimer = System.currentTimeMillis();
        }
        else if (automationState == AutomationState.READY && lastAutomationState != AutomationState.READY) {
            expandoDiag.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drop.setPosition(MarvConstantsV3.DROP_ANGLE_FLAT);
            swop.setPosition(MarvConstantsV3.SWOP_ANGLE_NORMAL);
        }
        else if (automationState == AutomationState.UP && lastAutomationState != AutomationState.UP) {
            expandoDiag.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (dropTarget == DropTarget.NEAR) {
                expandoDiag.setTargetPosition(MarvConstantsV3.EXPANDO_DIAG_UP_NEAR_POSITION);
            }
            else if (dropTarget == DropTarget.FAR) {
                expandoDiag.setTargetPosition(MarvConstantsV3.EXPANDO_DIAG_UP_FAR_POSITION);
            }
            else if (dropTarget == DropTarget.MID) {
                expandoDiag.setTargetPosition(MarvConstantsV3.EXPANDO_DIAG_UP_MID_POSITION);
            }
            expandoDiag.setPower(MarvConstantsV3.EXPANDO_DIAG_UP_POWER);
        }
        else if (automationState == AutomationState.DROP && lastAutomationState != AutomationState.DROP) {
            expandoDiag.setPower(0);
            if (dropTarget == DropTarget.NEAR) {
                drop.setPosition(MarvConstantsV3.DROP_ANGLE_NEAR);
            }
            else if (dropTarget == DropTarget.MID) {
                drop.setPosition(MarvConstantsV3.DROP_ANGLE_MID);
            }
            else if (dropTarget == DropTarget.FAR) {
                drop.setPosition(MarvConstantsV3.DROP_ANGLE_FAR);
                swop.setPosition(MarvConstantsV3.SWOP_ANGLE_SWOPPED);
            }
            dropTimer = System.currentTimeMillis();
        }
        else if (automationState == AutomationState.UNDROP && lastAutomationState != AutomationState.UNDROP) {
            expandoDiag.setPower(0);
            drop.setPosition(MarvConstantsV3.DROP_ANGLE_FLAT);
            swop.setPosition(MarvConstantsV3.SWOP_ANGLE_NORMAL);
            undropTimer = System.currentTimeMillis();
        }
        else if (automationState == AutomationState.DOWN && lastAutomationState != AutomationState.DOWN) {
            expandoDiag.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            expandoDiag.setTargetPosition(MarvConstantsV3.EXPANDO_DIAG_SAFE_POSITION);
            expandoDiag.setPower(MarvConstantsV3.EXPANDO_DIAG_SAFE_POWER);
        }


        if (automationState == AutomationState.STALIN) {
            if (System.currentTimeMillis() > stalinTimer + MarvConstantsV3.EXPANDO_DIAG_STALIN_TIME) {
                automationState = AutomationState.READY;
            }
        }
        else if (automationState == AutomationState.READY) {
            if (goStart) {
                automationState = AutomationState.UP;
            }
        }
        else if (automationState == AutomationState.UP) {
            if (!expandoDiag.isBusy() && goDrop) {
                automationState = AutomationState.DROP;
            }
        }
        else if (automationState == AutomationState.DROP) {
            if (System.currentTimeMillis() > dropTimer + MarvConstantsV3.EXPANDO_DIAG_DROP_TIME) {
                automationState = AutomationState.UNDROP;
            }
        }
        else if (automationState == AutomationState.UNDROP) {
            if (System.currentTimeMillis() > undropTimer + MarvConstantsV3.EXPANDO_DIAG_UNDROP_TIME) {
                automationState = AutomationState.DOWN;
            }
        }
        else if (automationState == AutomationState.DOWN) {
            if (!expandoDiag.isBusy()) {
                automationState = AutomationState.STALIN;
            }
        }

        lastAutomationState = automationState;
    }

    long dropTimer;
    long undropTimer;
    long stalinTimer;

}
