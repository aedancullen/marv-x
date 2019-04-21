package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.AnalogInput;

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

    AnalogInput dist;

    DcMotor.ZeroPowerBehavior lastZeroPowerBehavior;

    public enum AutomationState{STALIN, READY, UP, UPHOLD, DROP, UNDROP, DOWN, PAUSE}
    public enum DropTarget{FAR, NEAR, MID}

    public DropTarget dropTarget = DropTarget.NEAR;

    public AutomationState automationState = AutomationState.PAUSE;
    public AutomationState lastAutomationState;

    // -----

    public enum IntakeState{PREP, HUMAN, IN1, IN2, TRANSFER}
    public IntakeState intakeState = IntakeState.PREP;
    public IntakeState lastIntakeState;

    public boolean transferDone; // automation link flag

    public double dropGoingToPosition;
    public double swopGoingToPosition;

    public DcMotor getQuadPacerMotorX() {
        return br;
    }
    public DcMotor getQuadPacerMotorY() {
        return bl;
    }

    public boolean expandoVertCanDrop() {
        return (expandoVert.getCurrentPosition() > -MarvConstantsV3.EXPANDO_VERT_STOP);
    }

    public boolean expandoVertCanLift() {
        return (expandoVert.getCurrentPosition() < -5);
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
        dist = hardwareMap.analogInput.get("dist");

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
        expandoDiag.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (isAuto) {expandoDiag.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
        expandoDiag.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        expandoDiag.setPower(0);


        horizLiftL = hardwareMap.servo.get("horizLiftL");
        setServoExtendedRange(horizLiftL, 500, 2500);
        horizLiftL.setDirection(Servo.Direction.REVERSE);
        horizLiftR = hardwareMap.servo.get("horizLiftR");
        setServoExtendedRange(horizLiftR, 500, 2500);

        horizSpinL = hardwareMap.crservo.get("horizSpinL");
        horizSpinR = hardwareMap.crservo.get("horizSpinR");
        horizSpinR.setDirection(DcMotorSimple.Direction.REVERSE);

        drop = hardwareMap.servo.get("drop");
        setServoExtendedRange(drop, 500, 2500);

        swop = hardwareMap.servo.get("swop");
        swop.setDirection(Servo.Direction.REVERSE);
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
            expandoDiag.setPower(-MarvConstantsV3.EXPANDO_DIAG_STALIN_POWER);
            stalinTimer = System.currentTimeMillis();
            dCount = 0;
        }
        else if (automationState == AutomationState.READY && lastAutomationState != AutomationState.READY) {
            expandoDiag.setPower(0);
            expandoDiag.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drop.setPosition(MarvConstantsV3.DROP_ANGLE_FLAT);
            swop.setPosition(MarvConstantsV3.SWOP_ANGLE_NORMAL);
        }
        else if (automationState == AutomationState.UP && lastAutomationState != AutomationState.UP) {
            expandoDiag.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (dropTarget == DropTarget.NEAR) {
                expandoDiagUpChosenPosition = MarvConstantsV3.EXPANDO_DIAG_UP_NEAR_POSITION;
            }
            else if (dropTarget == DropTarget.FAR) {
                expandoDiagUpChosenPosition = MarvConstantsV3.EXPANDO_DIAG_UP_FAR_POSITION;
            }
            else if (dropTarget == DropTarget.MID) {
                expandoDiagUpChosenPosition = MarvConstantsV3.EXPANDO_DIAG_UP_MID_POSITION;
            }

            expandoDiag.setPower(MarvConstantsV3.EXPANDO_DIAG_UP_POWER);
        }
        else if (automationState == AutomationState.DROP && lastAutomationState != AutomationState.DROP) {
            if (dropTarget == DropTarget.NEAR) {
                //drop.setPosition(MarvConstantsV3.DROP_ANGLE_NEAR);
                dropGoingToPosition = MarvConstantsV3.DROP_ANGLE_NEAR;
                swopGoingToPosition = MarvConstantsV3.SWOP_ANGLE_NORMAL;
            }
            else if (dropTarget == DropTarget.MID) {
                //drop.setPosition(MarvConstantsV3.DROP_ANGLE_MID);
                dropGoingToPosition = MarvConstantsV3.DROP_ANGLE_MID;
                swopGoingToPosition = MarvConstantsV3.SWOP_ANGLE_NORMAL;
            }
            else if (dropTarget == DropTarget.FAR) {
                //drop.setPosition(MarvConstantsV3.DROP_ANGLE_FAR);
                dropGoingToPosition = MarvConstantsV3.DROP_ANGLE_FAR;
                //swop.setPosition(MarvConstantsV3.SWOP_ANGLE_SWOPPED);
                swopGoingToPosition = MarvConstantsV3.SWOP_ANGLE_SWOPPED;
            }
            dropTimer = System.currentTimeMillis();
        }
        else if (automationState == AutomationState.UNDROP && lastAutomationState != AutomationState.UNDROP) {
            //drop.setPosition(MarvConstantsV3.DROP_ANGLE_FLAT);
            dropGoingToPosition = MarvConstantsV3.DROP_ANGLE_FLAT;
            swop.setPosition(MarvConstantsV3.SWOP_ANGLE_NORMAL);
            undropTimer = System.currentTimeMillis();
        }
        else if (automationState == AutomationState.DOWN && lastAutomationState != AutomationState.DOWN) {
            drop.setPosition(MarvConstantsV3.DROP_ANGLE_FLAT);
            expandoDiag.setPower(0);
            expandoDiag.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            expandoDiag.setPower(-MarvConstantsV3.EXPANDO_DIAG_SAFE_POWER);
        }
        else if (automationState == AutomationState.PAUSE && lastAutomationState != AutomationState.PAUSE) {
            expandoDiag.setPower(0);
            pauseTimer = System.currentTimeMillis();
        }

        lastAutomationState = automationState;


        if (automationState == AutomationState.STALIN) {
            int diagPosition = expandoDiag.getCurrentPosition();
            if (lastDiagPosition == diagPosition) {
                dCount += 1;
            }
            else {
                dCount = 0;
            }
            if (dCount > 25 && System.currentTimeMillis() > stalinTimer + MarvConstantsV3.EXPANDO_DIAG_STALIN_TIME) {
                automationState = AutomationState.READY;
            }
            lastDiagPosition = diagPosition;
        }
        else if (automationState == AutomationState.READY) {
            if (goStart) {
                automationState = AutomationState.UP;
            }
        }
        else if (automationState == AutomationState.UP) {
            if (expandoDiag.getCurrentPosition() >= expandoDiagUpChosenPosition) {
                expandoDiag.setPower(0);
                int posAtStop = expandoDiag.getCurrentPosition();
                expandoDiag.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                expandoDiag.setTargetPosition(posAtStop + 50); // bleh
                expandoDiag.setPower(1);
                automationState = AutomationState.UPHOLD;
            }
        }
        else if (automationState == AutomationState.UPHOLD) {
            if (goDrop) {
                automationState = AutomationState.DROP;
            }
            if (dropTarget == DropTarget.FAR) {
                swop.setPosition(MarvConstantsV3.SWOP_ANGLE_SWOPPED);
                drop.setPosition(0.55);
            }
        }
        else if (automationState == AutomationState.DROP) {
            if (System.currentTimeMillis() > dropTimer + MarvConstantsV3.EXPANDO_DIAG_DROP_TIME) {
                automationState = AutomationState.UNDROP;
            }
            double dropPosition = drop.getPosition();
            if (dropPosition < dropGoingToPosition) {
                drop.setPosition(dropPosition + Math.min(dropGoingToPosition-dropPosition, MarvConstantsV3.EXPANDO_DIAG_DROP_RATE));
            } 
            double swopPosition = swop.getPosition();
            if (swopPosition < swopGoingToPosition) {
                swop.setPosition(swopPosition + Math.min(swopGoingToPosition-swopPosition, MarvConstantsV3.EXPANDO_DIAG_DROP_RATE));
            }
        }
        else if (automationState == AutomationState.UNDROP) {
            if (System.currentTimeMillis() > undropTimer + MarvConstantsV3.EXPANDO_DIAG_UNDROP_TIME) {
                automationState = AutomationState.DOWN;
            }
            double dropPosition = drop.getPosition();
            if (dropPosition > dropGoingToPosition) {
                drop.setPosition(dropPosition - Math.min(dropPosition-dropGoingToPosition, MarvConstantsV3.EXPANDO_DIAG_UNDROP_RATE));
            }
        }
        else if (automationState == AutomationState.DOWN) {
            if (expandoDiag.getCurrentPosition() <= MarvConstantsV3.EXPANDO_DIAG_SAFE_POSITION) {
                automationState = AutomationState.PAUSE;
            }
        }
        else if (automationState == AutomationState.PAUSE) {
            if (System.currentTimeMillis() > pauseTimer + MarvConstantsV3.EXPANDO_DIAG_PAUSE_TIME) {
                automationState = AutomationState.STALIN;
            }
        }

    }

    enum HumanState {UP, DOWN}
    HumanState humanState;

    public void runIntakeAutomation(double humanSlide, boolean humanUp, boolean humanDown, boolean humanStop, boolean humanEject, boolean go) {
        transferDone = false;

        if (intakeState == IntakeState.PREP && lastIntakeState != IntakeState.PREP) {
            expandoHorizL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            expandoHorizR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            setExpandoHorizRelative(MarvConstantsV3.EXPANDO_HORIZ_DOWN);
            expandoHorizL.setPower(0.25);
            expandoHorizR.setPower(0.25);
            horizLiftL.setPosition(MarvConstantsV3.HORIZ_LIFT_UP_WAITING);
            horizLiftR.setPosition(MarvConstantsV3.HORIZ_LIFT_UP_WAITING);
            horizSpinL.setPower(MarvConstantsV3.UC_HORIZSPIN_HOLD);
            horizSpinR.setPower(MarvConstantsV3.UC_HORIZSPIN_HOLD);
            transferDone = true;
        }
        else if (intakeState == IntakeState.HUMAN && lastIntakeState != IntakeState.HUMAN) {
            expandoHorizL.setPower(0);
            expandoHorizL.setPower(0);
            humanState = HumanState.UP;
            expandoHorizL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            expandoHorizR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //transferDone = true;
        }
        else if (intakeState == IntakeState.IN1 && lastIntakeState != IntakeState.IN1) {
            expandoHorizL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            expandoHorizR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            setExpandoHorizRelative(MarvConstantsV3.EXPANDO_HORIZ_SAFE);
            expandoHorizL.setPower(1);
            expandoHorizR.setPower(1);
        }
        else if (intakeState == IntakeState.IN2 && lastIntakeState != IntakeState.IN2) {
            in2Timer = System.currentTimeMillis();
            horizLiftL.setPosition(MarvConstantsV3.HORIZ_LIFT_UP_DUMPING);
            horizLiftR.setPosition(MarvConstantsV3.HORIZ_LIFT_UP_DUMPING);
        }
        else if (intakeState == IntakeState.TRANSFER && lastIntakeState != IntakeState.TRANSFER) {
            transferTimer = System.currentTimeMillis();
            horizSpinL.setPower(-MarvConstantsV3.UC_HORIZSPIN_TRANSFER);
            horizSpinR.setPower(-MarvConstantsV3.UC_HORIZSPIN_TRANSFER);
        }

        lastIntakeState = intakeState;

        if (intakeState == IntakeState.PREP) {
            boolean done = (!expandoHorizL.isBusy() || !expandoHorizR.isBusy());
            if (done) {
                expandoHorizL.setPower(0);
                expandoHorizR.setPower(0);
                intakeState = IntakeState.HUMAN;
            }
        }
        else if (intakeState == IntakeState.HUMAN) { if (!transferDone && automationState != AutomationState.UP && automationState != AutomationState.UPHOLD && automationState != AutomationState.DROP) {

            if (go) {
                intakeState = IntakeState.IN1;
                humanState = HumanState.DOWN;
            }

            int liftState = -1;
            double pos = (expandoHorizL.getCurrentPosition() + expandoHorizR.getCurrentPosition()) / 2.0;
            if (Math.abs(horizLiftL.getPosition() - MarvConstantsV3.HORIZ_LIFT_DOWN) < 0.01) {
                liftState = 0;
            }
            else if (Math.abs(horizLiftL.getPosition() - MarvConstantsV3.HORIZ_LIFT_UP_NEUTRAL) < 0.01) {
                liftState = 1;
            }
            else if (Math.abs(horizLiftL.getPosition() - MarvConstantsV3.HORIZ_LIFT_UP_WAITING) < 0.01) {
                liftState = 2;
            }

            if (humanSlide > 0 && pos < MarvConstantsV3.EXPANDO_HORIZ_UP) {
                if (liftState == 2) {
                    horizLiftL.setPosition(MarvConstantsV3.HORIZ_LIFT_UP_NEUTRAL);
                    horizLiftR.setPosition(MarvConstantsV3.HORIZ_LIFT_UP_NEUTRAL);
                }

                if (pos > MarvConstantsV3.EXPANDO_HORIZ_FLYING_LIMIT && liftState != 0) {
                    expandoHorizL.setPower(0);
                    expandoHorizR.setPower(0);
                }
                else {
                    expandoHorizL.setPower(humanSlide);
                    expandoHorizR.setPower(humanSlide);
                }
            }
            else if (humanSlide < 0 && pos > MarvConstantsV3.EXPANDO_HORIZ_DOWN + MarvConstantsV3.UC_EXPANDOHORIZ_BUF) {
                expandoHorizL.setPower(humanSlide);
                expandoHorizR.setPower(humanSlide);
            }
            else {
                expandoHorizL.setPower(0);
                expandoHorizR.setPower(0);
            }

            if (humanUp && pos < MarvConstantsV3.EXPANDO_HORIZ_FLYING_LIMIT) {
                humanState = HumanState.UP;
                horizLiftL.setPosition(MarvConstantsV3.HORIZ_LIFT_UP_NEUTRAL);
                horizLiftR.setPosition(MarvConstantsV3.HORIZ_LIFT_UP_NEUTRAL);
            }
            if (humanDown) {
                humanState = HumanState.DOWN;
                horizLiftL.setPosition(MarvConstantsV3.HORIZ_LIFT_DOWN);
                horizLiftR.setPosition(MarvConstantsV3.HORIZ_LIFT_DOWN);
            }

            if (humanStop) {
                horizSpinL.setPower(0);
                horizSpinR.setPower(0);
            }
            else if (humanEject) {
                horizSpinL.setPower(-MarvConstantsV3.UC_HORIZSPIN_EJECT);
                horizSpinR.setPower(-MarvConstantsV3.UC_HORIZSPIN_EJECT);
            }
            else if (humanState == HumanState.DOWN) {
                horizSpinL.setPower(MarvConstantsV3.UC_HORIZSPIN_INTAKE);
                horizSpinR.setPower(MarvConstantsV3.UC_HORIZSPIN_INTAKE);
            }
            else if (humanState == HumanState.UP) {
                horizSpinL.setPower(MarvConstantsV3.UC_HORIZSPIN_HOLD);
                horizSpinR.setPower(MarvConstantsV3.UC_HORIZSPIN_HOLD);
            }
        }}
        else if (intakeState == IntakeState.IN1) {
            if ((expandoHorizL.getCurrentPosition() + expandoHorizR.getCurrentPosition()) / 2.0 < MarvConstantsV3.EXPANDO_HORIZ_FLYING_LIMIT) {
                intakeState = IntakeState.IN2;
            }
        }
        else if (intakeState == IntakeState.IN2) {
            boolean done = (!expandoHorizL.isBusy() || !expandoHorizR.isBusy());
            if (done) {
                expandoHorizL.setPower(0);
                expandoHorizR.setPower(0);
            }
            if (expandoHorizL.getPower() == 0 && System.currentTimeMillis() > in2Timer + MarvConstantsV3.EHSM_UP && automationState == AutomationState.READY) {
                intakeState = IntakeState.TRANSFER;
            }
        }
        else if (intakeState == IntakeState.TRANSFER) {
            if (System.currentTimeMillis() > transferTimer + MarvConstantsV3.EHSM_TRANSFER) {
                intakeState = IntakeState.PREP;
            }
        }
    }

    private void setExpandoHorizRelative(int target) {
        int avg = (int)((expandoHorizL.getCurrentPosition() + expandoHorizR.getCurrentPosition()) / 2.0);
        int d = target - avg;
        expandoHorizL.setTargetPosition(expandoHorizL.getCurrentPosition() + d);
        expandoHorizR.setTargetPosition(expandoHorizR.getCurrentPosition() + d);
    }

    long dropTimer;
    long undropTimer;
    long stalinTimer;

    long pauseTimer;

    int expandoDiagUpChosenPosition;
    int lastDiagPosition;

    int dCount;

    // -----

    long in2Timer;
    long transferTimer;
}
