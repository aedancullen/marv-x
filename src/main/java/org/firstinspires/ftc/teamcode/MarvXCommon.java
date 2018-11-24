package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;


public class MarvXCommon {

    BNO055IMU imu;

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    DcMotor expandoHoriz;
    DcMotor expandoVertL;
    DcMotor expandoVertR;

    DcMotor horizSpin;
    Servo horizBoxL;
    Servo horizBoxR;

    Servo vertBoxL;
    Servo vertBoxR;
    Servo vertSpin;

    enum IntakeState {UP_NEUTRAL, UP_DUMPING, DOWN_NEUTRAL, DOWN_PAUSED, DOWN_EJECTING,   STROBE_UP,STROBE_DOWN}
    enum ExpandoHorizState {STROBE_IN, MANUAL}

    enum ExpandoVertState {UP, DOWN, SAFE}
    enum BoxLiftState {UP, DOWN, SAFE}

    enum BoxSpinState {RESET, SET_BACK, SET_DROP}

    enum BackTarget {RIGHT, LEFT}
    enum DropTarget {FARTHER, LESSER, NEUTRAL}

    enum AutomationState {HUMAN_INTAKE, HORIZONTAL, TRANSFER, VERTICAL, HUMAN_DROP, RESET}

    IntakeState intakeState = IntakeState.UP_NEUTRAL;
    ExpandoHorizState expandoHorizState = ExpandoHorizState.MANUAL;
    ExpandoHorizState lastExpandoHorizState = null;
    ExpandoVertState expandoVertState = ExpandoVertState.DOWN;
    BoxLiftState boxLiftState = BoxLiftState.SAFE;
    BoxSpinState boxSpinState = BoxSpinState.RESET;

    BackTarget backTarget = null;
    DropTarget dropTarget = null;

    AutomationState automationState = AutomationState.HUMAN_INTAKE;


    DcMotor.ZeroPowerBehavior lastZeroPowerBehavior;

    public MarvXCommon(HardwareMap hardwareMap) {
        fl = hardwareMap.dcMotor.get("fl");

        fr = hardwareMap.dcMotor.get("fr");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);

        bl = hardwareMap.dcMotor.get("bl");

        br = hardwareMap.dcMotor.get("br");
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        this.setEncoderBehavior(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setEncoderBehavior(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        expandoHoriz = hardwareMap.dcMotor.get("expandoHoriz");
        expandoHoriz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        expandoHoriz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        expandoHoriz.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        expandoVertL = hardwareMap.dcMotor.get("expandoVertL");
        expandoVertL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        expandoVertL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        expandoVertL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        expandoVertR = hardwareMap.dcMotor.get("expandoVertR");
        expandoVertR.setDirection(DcMotorSimple.Direction.REVERSE);
        expandoVertR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        expandoVertR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        expandoVertR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        horizSpin = hardwareMap.dcMotor.get("horizSpin");
        horizSpin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizSpin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ONE REVERSE
        horizBoxL = hardwareMap.servo.get("horizBoxL");
        horizBoxR = hardwareMap.servo.get("horizBoxR");

        // ONE REVERSE
        vertBoxL = hardwareMap.servo.get("vertBoxL");
        setServoExtendedRange(vertBoxL, 500, 2500);
        vertBoxR = hardwareMap.servo.get("vertBoxR");
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


    public void runIntakeStateMachine() {
        if (intakeState == IntakeState.UP_NEUTRAL) {
            horizBoxL.setPosition(MarvNavConstants.HORIZ_BOX_UP_NEUTRAL);
            horizBoxR.setPosition(MarvNavConstants.HORIZ_BOX_UP_NEUTRAL);
            if(horizSpin.getCurrentPosition() % MarvNavConstants.HORIZ_SPIN_CLEAR_MODULUS > MarvNavConstants.HORIZ_SPIN_MODULUS_TOLERANCE){
                horizSpin.setPower(0.2);
            }
            else{horizSpin.setPower(0);}
        }
        else if (intakeState == IntakeState.UP_DUMPING) {
            horizBoxL.setPosition(MarvNavConstants.HORIZ_BOX_UP_DUMPING);
            horizBoxR.setPosition(MarvNavConstants.HORIZ_BOX_UP_DUMPING);
            horizSpin.setPower(0);
        }
        else if (intakeState == IntakeState.DOWN_NEUTRAL) {
            horizBoxL.setPosition(MarvNavConstants.HORIZ_BOX_DOWN);
            horizBoxR.setPosition(MarvNavConstants.HORIZ_BOX_DOWN);
            horizSpin.setPower(1);
        }
        else if (intakeState == IntakeState.DOWN_PAUSED) {
            horizBoxL.setPosition(MarvNavConstants.HORIZ_BOX_DOWN);
            horizBoxR.setPosition(MarvNavConstants.HORIZ_BOX_DOWN);
            horizSpin.setPower(0);
        }
        else if (intakeState == IntakeState.DOWN_EJECTING) {
            horizBoxL.setPosition(MarvNavConstants.HORIZ_BOX_DOWN);
            horizBoxR.setPosition(MarvNavConstants.HORIZ_BOX_DOWN);
            horizSpin.setPower(-1);
        }
        else if (intakeState == IntakeState.STROBE_UP) {
            if(horizSpin.getCurrentPosition() % MarvNavConstants.HORIZ_SPIN_READY_MODULUS > MarvNavConstants.HORIZ_SPIN_MODULUS_TOLERANCE){
                horizSpin.setPower(1);
            }
            else{
                horizSpin.setPower(0);
                intakeState = IntakeState.UP_NEUTRAL;
            }
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

            if (System.currentTimeMillis() > strobeDownStartTime + 200){intakeState = IntakeState.DOWN_NEUTRAL;}
        }
    }

    long strobeDownStartTime = -1;

    public void runExpandoHorizStateMachine(float manualPower) {
        if (lastExpandoHorizState == ExpandoHorizState.MANUAL && expandoHorizState == ExpandoHorizState.STROBE_IN) {
            expandoHoriz.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            expandoHoriz.setTargetPosition(0);
            expandoHoriz.setPower(1);
        }
        else if (expandoHorizState == ExpandoHorizState.MANUAL) {
            if (manualPower > 0 && expandoHoriz.getCurrentPosition() < MarvNavConstants.EXPANDO_HORIZ_LIMIT) {
                expandoHoriz.setPower(manualPower);
            }
            if (manualPower < 0 && expandoHoriz.getCurrentPosition() > 0) {
                expandoHoriz.setPower(manualPower);
            }
            else {
                expandoHoriz.setPower(0);
            }
        }
        else if (expandoHorizState == ExpandoHorizState.STROBE_IN) {
            if (!expandoHoriz.isBusy()) {
                expandoHoriz.setPower(0);
                expandoHoriz.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                expandoHorizState = ExpandoHorizState.MANUAL;
            }
        }

        lastExpandoHorizState = expandoHorizState;
    }


}
