package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class MarvXCommon {

    BNO055IMU imu;

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    public DcMotor expandoHoriz;
    public DcMotor expandoVertL;
    public DcMotor expandoVertR;

    DcMotor horizSpin;
    Servo horizBoxL;
    Servo horizBoxR;

    Servo vertBoxL;
    Servo vertBoxR;
    Servo vertSpin;

    ColorSensor color1;
    DistanceSensor distance1;
    ColorSensor color2;
    DistanceSensor distance2;

    public enum IntakeState {UP_NEUTRAL, UP_DUMPING, DOWN_NEUTRAL,   STROBE_UP,STROBE_DOWN,    SAFE}
    public enum ExpandoHorizState {STROBE_DUMP, MANUAL}

    public enum ExpandoVertState {SET_UP, DOWN, SAFE}
    public enum BoxLiftState {SET_UP, DOWN, SAFE}

    public enum BoxSpinState {RESET, SET_BACK, SET_DROP}

    public enum BackTarget {RIGHT, LEFT}
    public enum DropTarget {FARTHER, LESSER, NEUTRAL}
    public enum ArmTarget {NEAR, BOUND, FAR}

    public enum AutomationState {HUMAN_INTAKE, HORIZONTAL, TRANSFER, VERTICAL, HUMAN_DROP}

    public IntakeState intakeState = IntakeState.SAFE;
    public ExpandoHorizState expandoHorizState = ExpandoHorizState.MANUAL;
    public ExpandoHorizState lastExpandoHorizState = null;
    public ExpandoVertState expandoVertState = ExpandoVertState.DOWN;
    public ExpandoVertState lastExpandoVertState = null;
    public BoxLiftState boxLiftState = BoxLiftState.SAFE;
    public BoxSpinState boxSpinState = BoxSpinState.RESET;

    BackTarget backTarget = null;
    DropTarget dropTarget = null;
    ArmTarget armTarget = null;

    public AutomationState automationState = AutomationState.HUMAN_INTAKE;


    DcMotor.ZeroPowerBehavior lastZeroPowerBehavior;

    public MarvXCommon(HardwareMap hardwareMap, boolean maintainFromAuto) {

        if (maintainFromAuto) {
            intakeState = IntakeState.UP_NEUTRAL;
            boxLiftState = BoxLiftState.DOWN;
        }

        color1 = hardwareMap.get(ColorSensor.class, "cd1");
        distance1 = hardwareMap.get(DistanceSensor.class, "cd1");
        color2 = hardwareMap.get(ColorSensor.class, "cd2");
        distance2 = hardwareMap.get(DistanceSensor.class, "cd2");

        fl = hardwareMap.dcMotor.get("fl");

        fr = hardwareMap.dcMotor.get("fr");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);

        bl = hardwareMap.dcMotor.get("bl");

        br = hardwareMap.dcMotor.get("br");
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        if (!maintainFromAuto) {this.setEncoderBehavior(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
        this.setEncoderBehavior(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        expandoHoriz = hardwareMap.dcMotor.get("expandoHoriz");
        expandoHoriz.setDirection(DcMotorSimple.Direction.REVERSE);
        expandoHoriz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (!maintainFromAuto) {expandoHoriz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
        expandoHoriz.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        expandoVertL = hardwareMap.dcMotor.get("expandoVertL");
        expandoVertL.setDirection(DcMotorSimple.Direction.REVERSE);
        expandoVertL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (!maintainFromAuto) {expandoVertL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
        expandoVertL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        expandoVertL.setTargetPosition(MarvNavConstants.EXPANDO_VERT_DOWN);
        expandoVertL.setPower(1);

        expandoVertR = hardwareMap.dcMotor.get("expandoVertR");
        expandoVertR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (!maintainFromAuto) {expandoVertR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
        expandoVertR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        expandoVertR.setTargetPosition(MarvNavConstants.EXPANDO_VERT_DOWN);
        expandoVertR.setPower(1);

        horizSpin = hardwareMap.dcMotor.get("horizSpin");
        horizSpin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (!maintainFromAuto) {horizSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
        horizSpin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ONE REVERSE
        horizBoxL = hardwareMap.servo.get("horizBoxL");
        setServoExtendedRange(horizBoxL, 500, 2500);
        horizBoxR = hardwareMap.servo.get("horizBoxR");
        horizBoxR.setDirection(Servo.Direction.REVERSE);
        setServoExtendedRange(horizBoxR, 500, 2500);

        // ONE REVERSE
        vertBoxL = hardwareMap.servo.get("vertBoxL");
        setServoExtendedRange(vertBoxL, 500, 2500);
        vertBoxR = hardwareMap.servo.get("vertBoxR");
        vertBoxR.setDirection(Servo.Direction.REVERSE);
        setServoExtendedRange(vertBoxR, 500, 2500);

        vertSpin = hardwareMap.servo.get("vertSpin");
        setServoExtendedRange(vertSpin, 500, 2500);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu.initialize(parameters);
    }

    public void setServoExtendedRange(Servo servo, int min, int max) {
        ServoControllerEx controller = (ServoControllerEx) servo.getController();
        int servoPort = servo.getPortNumber();
        PwmControl.PwmRange range = new PwmControl.PwmRange(min, max);
        controller.setServoPwmRange(servoPort, range);
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

    public double getQuadPacerX() {
        return fl.getCurrentPosition();
    }
    public double getQuadPacerY() {
        return bl.getCurrentPosition();
    }

    public DcMotor getQuadPacerMotorX() {
        return fl;
    }
    public DcMotor getQuadPacerMotorY() {
        return bl;
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


    public void runIntakeStateMachine(float manualPower) {
        if (intakeState == IntakeState.SAFE) {
            horizBoxL.setPosition(MarvNavConstants.HORIZ_BOX_SAFE);
            horizBoxR.setPosition(MarvNavConstants.HORIZ_BOX_SAFE);
        }


        else if (intakeState == IntakeState.UP_NEUTRAL) {
            horizBoxL.setPosition(MarvNavConstants.HORIZ_BOX_UP_NEUTRAL);
            horizBoxR.setPosition(MarvNavConstants.HORIZ_BOX_UP_NEUTRAL);
            /*if(horizSpin.getCurrentPosition() % MarvNavConstants.HORIZ_SPIN_CLEAR_MODULUS > MarvNavConstants.HORIZ_SPIN_MODULUS_TOLERANCE){
                horizSpin.setPower(0.5);
            }
            else{horizSpin.setPower(0);}*/
        }
        else if (intakeState == IntakeState.UP_DUMPING) {
            horizBoxL.setPosition(MarvNavConstants.HORIZ_BOX_UP_DUMPING);
            horizBoxR.setPosition(MarvNavConstants.HORIZ_BOX_UP_DUMPING);
            horizSpin.setPower(0);
        }
        else if (intakeState == IntakeState.DOWN_NEUTRAL) {
            horizBoxL.setPosition(MarvNavConstants.HORIZ_BOX_DOWN);
            horizBoxR.setPosition(MarvNavConstants.HORIZ_BOX_DOWN);
            horizSpin.setPower(manualPower);
        }
        else if (intakeState == IntakeState.STROBE_UP) {
            if (strobeUpStartTime == -1) {
                strobeUpStartTime = System.currentTimeMillis();
            }
            horizBoxL.setPosition(MarvNavConstants.HORIZ_BOX_UP_NEUTRAL);
            horizBoxR.setPosition(MarvNavConstants.HORIZ_BOX_UP_NEUTRAL);
            if(horizSpin.getCurrentPosition() % MarvNavConstants.HORIZ_SPIN_READY_MODULUS > MarvNavConstants.HORIZ_SPIN_MODULUS_TOLERANCE){
                horizSpin.setPower(1);
            }
            else{
                horizSpin.setPower(0);
            }

            if (System.currentTimeMillis() > strobeUpStartTime + MarvNavConstants.HORIZ_BOX_TOUP_MILLIS){intakeState = IntakeState.UP_NEUTRAL;}
        }
        else if (intakeState == IntakeState.STROBE_DOWN) {
            if (strobeDownStartTime == -1) {
                strobeDownStartTime = System.currentTimeMillis();
            }
            horizBoxL.setPosition(MarvNavConstants.HORIZ_BOX_DOWN);
            horizBoxR.setPosition(MarvNavConstants.HORIZ_BOX_DOWN);
            if(horizSpin.getCurrentPosition() % MarvNavConstants.HORIZ_SPIN_READY_MODULUS > MarvNavConstants.HORIZ_SPIN_MODULUS_TOLERANCE){
                horizSpin.setPower(-1);
            }
            else{
                horizSpin.setPower(0);
            }

            if (System.currentTimeMillis() > strobeDownStartTime + MarvNavConstants.HORIZ_BOX_TODOWN_MILLIS){intakeState = IntakeState.DOWN_NEUTRAL;}
        }


        if (intakeState != IntakeState.STROBE_UP) {strobeUpStartTime = -1;}
        if (intakeState != IntakeState.STROBE_DOWN) {strobeDownStartTime = -1;}
    }

    long strobeDownStartTime = -1;
    long strobeUpStartTime = -1;

    public void runExpandoHorizStateMachine(float manualPower) {
        if (lastExpandoHorizState == ExpandoHorizState.MANUAL && expandoHorizState == ExpandoHorizState.STROBE_DUMP) {
            expandoHoriz.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            expandoHoriz.setTargetPosition(MarvNavConstants.EXPANDO_HORIZ_DUMP);
            expandoHoriz.setPower(0.5);
        }
        else if (lastExpandoHorizState == ExpandoHorizState.STROBE_DUMP && expandoHorizState == ExpandoHorizState.MANUAL) {
            expandoHoriz.setPower(0);
            expandoHoriz.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        lastExpandoHorizState = expandoHorizState;

        if (expandoHorizState == ExpandoHorizState.MANUAL) {
            if (manualPower > 0 && expandoHoriz.getCurrentPosition() < MarvNavConstants.EXPANDO_HORIZ_LIMIT) {
                expandoHoriz.setPower(manualPower);
            }
            else if (manualPower < 0 && expandoHoriz.getCurrentPosition() > 0) {
                expandoHoriz.setPower(manualPower);
            }
            else {
                expandoHoriz.setPower(0);
            }
        }
        else if (expandoHorizState == ExpandoHorizState.STROBE_DUMP) {
            if (!expandoHoriz.isBusy()) {
                expandoHorizState = ExpandoHorizState.MANUAL;
            }
        }
    }


    public void runExpandoVertStateMachine() {
        if (lastExpandoVertState != ExpandoVertState.SET_UP && expandoVertState == ExpandoVertState.SET_UP) {
            expandoVertL.setPower(MarvNavConstants.EXPANDO_VERT_TOUP_SPEED);
            expandoVertR.setPower(MarvNavConstants.EXPANDO_VERT_TOUP_SPEED);
            expandoVertL.setTargetPosition(MarvNavConstants.EXPANDO_VERT_UP_FAR);
            expandoVertR.setTargetPosition(MarvNavConstants.EXPANDO_VERT_UP_FAR);
        }
        else if (lastExpandoVertState != ExpandoVertState.DOWN && expandoVertState == ExpandoVertState.DOWN) {
            expandoVertL.setPower(MarvNavConstants.EXPANDO_VERT_TODOWN_SPEED);
            expandoVertR.setPower(MarvNavConstants.EXPANDO_VERT_TODOWN_SPEED);
            expandoVertL.setTargetPosition(MarvNavConstants.EXPANDO_VERT_DOWN);
            expandoVertR.setTargetPosition(MarvNavConstants.EXPANDO_VERT_DOWN);
        }
        else if (lastExpandoVertState != ExpandoVertState.SAFE && expandoVertState == ExpandoVertState.SAFE) {
            expandoVertL.setPower(MarvNavConstants.EXPANDO_VERT_TOSAFE_SPEED);
            expandoVertR.setPower(MarvNavConstants.EXPANDO_VERT_TOSAFE_SPEED);
            expandoVertL.setTargetPosition(MarvNavConstants.EXPANDO_VERT_SAFE);
            expandoVertR.setTargetPosition(MarvNavConstants.EXPANDO_VERT_SAFE);
        }

        lastExpandoVertState = expandoVertState;
    }


    public void runBoxLiftStateMachine() {
        if (boxLiftState == BoxLiftState.SET_UP) {
            vertBoxL.setPosition(MarvNavConstants.VERT_BOX_UP_FAR);
            vertBoxR.setPosition(MarvNavConstants.VERT_BOX_UP_FAR);
        }
        else if (boxLiftState == BoxLiftState.DOWN) {
            vertBoxL.setPosition(MarvNavConstants.VERT_BOX_DOWN);
            vertBoxR.setPosition(MarvNavConstants.VERT_BOX_DOWN);
        }
        else if (boxLiftState == BoxLiftState.SAFE) {
            vertBoxL.setPosition(MarvNavConstants.VERT_BOX_SAFE);
            vertBoxR.setPosition(MarvNavConstants.VERT_BOX_SAFE);
        }
    }

    public void runBoxSpinStateMachine() {
        if (boxSpinState == BoxSpinState.RESET) {
            vertSpin.setPosition(MarvNavConstants.VERT_SPIN_NEUTRAL);
        }
        else if (boxSpinState == BoxSpinState.SET_BACK) {
            if (backTarget == BackTarget.RIGHT) {
                vertSpin.setPosition(MarvNavConstants.VERT_SPIN_R2BACK);
            }
            else if (backTarget == BackTarget.LEFT) {
                vertSpin.setPosition(MarvNavConstants.VERT_SPIN_L2BACK);
            }
        }
        else if (boxSpinState == BoxSpinState.SET_DROP) {
            if (backTarget == BackTarget.RIGHT) {
                if (dropTarget == DropTarget.FARTHER) {vertSpin.setPosition(MarvNavConstants.VERT_SPIN_R2BACK_FARTHER);}
                else if (dropTarget == DropTarget.LESSER) {vertSpin.setPosition(MarvNavConstants.VERT_SPIN_R2BACK_LESSER);}
            }
            else if (backTarget == BackTarget.LEFT) {
                if (dropTarget == DropTarget.FARTHER) {vertSpin.setPosition(MarvNavConstants.VERT_SPIN_L2BACK_FARTHER);}
                else if (dropTarget == DropTarget.LESSER) {vertSpin.setPosition(MarvNavConstants.VERT_SPIN_L2BACK_LESSER);}
            }
        }
    }

    public void automationCancel() {
        expandoHorizState = ExpandoHorizState.MANUAL;
        expandoVertState = ExpandoVertState.DOWN;
        boxLiftState = BoxLiftState.DOWN;
        boxSpinState = BoxSpinState.RESET;
        automationState = AutomationState.HUMAN_INTAKE;
    }

    public void runAllMachinesNoInput() {
        runIntakeStateMachine(0);
        runExpandoHorizStateMachine(0);
        runExpandoVertStateMachine();
        runBoxLiftStateMachine();
        runBoxSpinStateMachine();
    }

    public void runAutomationStateMachine(boolean go, boolean cancel, float intakeManualPower, float expandoHorizManualPower) {
        runIntakeStateMachine(intakeManualPower);
        runExpandoHorizStateMachine(expandoHorizManualPower);
        runExpandoVertStateMachine();
        runBoxLiftStateMachine();
        runBoxSpinStateMachine();

        if (cancel) {automationCancel();}

        if (automationState == AutomationState.HUMAN_INTAKE) {
            if (go) {
                if (intakeState != IntakeState.UP_NEUTRAL && intakeState != IntakeState.STROBE_UP) {intakeState = IntakeState.STROBE_UP;}
                expandoHorizState = ExpandoHorizState.STROBE_DUMP;

                automationState = AutomationState.HORIZONTAL;
            }
        }
        else if (automationState == AutomationState.HORIZONTAL) {
            if (expandoHorizState == ExpandoHorizState.MANUAL) {
                intakeState = IntakeState.UP_DUMPING;

                automationState = AutomationState.TRANSFER;
            }
        }
        else if (automationState == AutomationState.TRANSFER) {
            if (distance1.getDistance(DistanceUnit.MM) < 60 && distance2.getDistance(DistanceUnit.MM) < 60) {
                intakeState = IntakeState.UP_NEUTRAL;
                expandoVertState = ExpandoVertState.SET_UP;

                double lWarmScore = (float)color1.red() / color1.blue();
                double rWarmScore = (float)color2.red() / color2.blue();

                if (lWarmScore < 1.5 && rWarmScore < 1.5) {
                    backTarget = BackTarget.LEFT;
                    dropTarget = DropTarget.NEUTRAL;
                }
                else if (lWarmScore > 1.5 && rWarmScore > 1.5) {
                    backTarget = BackTarget.LEFT;
                    dropTarget = DropTarget.LESSER;
                }
                else if (lWarmScore > 1.5 && rWarmScore < 1.5){
                    // Cube on left
                    backTarget = BackTarget.RIGHT;
                    dropTarget = DropTarget.LESSER;
                }
                else if (lWarmScore < 1.5 && rWarmScore > 1.5) {
                    // Cube on right
                    backTarget = BackTarget.LEFT;
                    dropTarget = DropTarget.FARTHER;
                }

                automationState = AutomationState.VERTICAL;
            }
        }
        else if (automationState == AutomationState.VERTICAL) {
            if ((expandoVertL.getCurrentPosition() + expandoVertR.getCurrentPosition()) / 2.0 > MarvNavConstants.EXPANDO_VERT_BOXFREE) {
                boxLiftState = BoxLiftState.SET_UP;
                boxSpinState = BoxSpinState.SET_BACK;
            }
            if (go) {
                boxSpinState = BoxSpinState.SET_DROP;

                humanDropStartTime = System.currentTimeMillis();
                automationState = AutomationState.HUMAN_DROP;
            }
        }
        else if (automationState == AutomationState.HUMAN_DROP) {
            if (System.currentTimeMillis() > humanDropStartTime + MarvNavConstants.VERT_SPIN_TODROP_MILLIS) {
                automationCancel();
            }
        }
    }

    long humanDropStartTime = -1;


}
