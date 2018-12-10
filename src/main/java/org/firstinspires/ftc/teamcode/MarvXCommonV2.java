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


public class MarvXCommonV2 {

    BNO055IMU imu;

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    public DcMotor expandoHorizL;
    public DcMotor expandoHorizR;
    public DcMotor expandoVertL;
    public DcMotor expandoVertR;

    Servo horizSpinL;
    Servo horizSpinR;
    Servo horizLiftL;
    Servo horizLiftR;

    double lastHorizLiftLPosition = -1;

    Servo vertLiftL;
    Servo vertLiftR;
    Servo vertSpin;

    ColorSensor color1;
    DistanceSensor distance1;
    ColorSensor color2;
    DistanceSensor distance2;

    public enum BackTarget {RIGHT, LEFT}
    public enum DropTarget {FARTHER, LESSER, NEUTRAL}
    public enum AutomationState {DOWN, FREE, UP, DROP};

    BackTarget backTarget = null;
    DropTarget dropTarget = null;
    public AutomationState automationState = AutomationState.DOWN;
    public AutomationState lastAutomationState = null;

    DcMotor.ZeroPowerBehavior lastZeroPowerBehavior;

    public MarvXCommonV2(HardwareMap hardwareMap, boolean maintainExpandoHoriz) {

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

        this.setEncoderBehavior(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setEncoderBehavior(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        expandoHorizL = hardwareMap.dcMotor.get("expandoHorizL");
        expandoHorizL.setDirection(DcMotorSimple.Direction.REVERSE);
        expandoHorizL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (!maintainExpandoHoriz) {expandoHorizL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
        expandoHorizL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        expandoHorizR = hardwareMap.dcMotor.get("expandoHorizR");
        expandoHorizR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (!maintainExpandoHoriz) {expandoHorizR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
        expandoHorizR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        expandoVertL = hardwareMap.dcMotor.get("expandoVertL");
        expandoVertL.setDirection(DcMotorSimple.Direction.REVERSE);
        expandoVertL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        expandoVertL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        expandoVertL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        expandoVertL.setTargetPosition(MarvNavConstants.EXPANDO_VERT_DOWN);
        expandoVertL.setPower(1);

        expandoVertR = hardwareMap.dcMotor.get("expandoVertR");
        expandoVertR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        expandoVertR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        expandoVertR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        expandoVertR.setTargetPosition(MarvNavConstants.EXPANDO_VERT_DOWN);
        expandoVertR.setPower(1);

        // ONE REVERSE
        horizLiftL = hardwareMap.servo.get("horizLiftL");
        setServoExtendedRange(horizLiftL, 500, 2500);
        horizLiftR = hardwareMap.servo.get("horizLiftR");
        horizLiftR.setDirection(Servo.Direction.REVERSE);
        setServoExtendedRange(horizLiftR, 500, 2500);

        // ONE REVERSE
        vertLiftL = hardwareMap.servo.get("vertLiftL");
        setServoExtendedRange(vertLiftL, 500, 2500);
        vertLiftR = hardwareMap.servo.get("vertLiftR");
        vertLiftR.setDirection(Servo.Direction.REVERSE);
        setServoExtendedRange(vertLiftR, 500, 2500);

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


    public void runAutomation(boolean drop, boolean fstart) {
        double horizLiftLPosition = horizLiftL.getPosition();


        if (automationState == AutomationState.DOWN) {
            expandoVertL.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_DOWN);
            expandoVertR.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_DOWN);
            expandoVertL.setPower(MarvConstantsV2.EXPANDO_VERT_TODOWN_SPEED);
            expandoVertR.setPower(MarvConstantsV2.EXPANDO_VERT_TODOWN_SPEED);
            vertLiftL.setPosition(MarvConstantsV2.VERT_LIFT_DOWN);
            vertLiftR.setPosition(MarvConstantsV2.VERT_LIFT_DOWN);

            if ((horizLiftL.getPosition() == MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL && lastHorizLiftLPosition == MarvConstantsV2.HORIZ_LIFT_UP_DUMPING) || fstart) {
                automationState = AutomationState.FREE;

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

            }
        }
        else if (automationState == AutomationState.FREE) {
            expandoVertL.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_BOXFREE);
            expandoVertR.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_BOXFREE);
            expandoVertL.setPower(MarvConstantsV2.EXPANDO_VERT_TOUP_SPEED);
            expandoVertR.setPower(MarvConstantsV2.EXPANDO_VERT_TOUP_SPEED);
            if (!expandoVertL.isBusy() || !expandoVertR.isBusy()) {
                automationState = AutomationState.UP;
            }
        }
        else if (automationState == AutomationState.UP) {
            expandoVertL.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_UP);
            expandoVertR.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_UP);
            expandoVertL.setPower(MarvConstantsV2.EXPANDO_VERT_TOUP_SPEED);
            expandoVertR.setPower(MarvConstantsV2.EXPANDO_VERT_TOUP_SPEED);
            vertLiftL.setPosition(MarvConstantsV2.VERT_LIFT_UP);
            vertLiftR.setPosition(MarvConstantsV2.VERT_LIFT_UP);

            if (backTarget == BackTarget.LEFT) {
                vertSpin.setPosition(MarvConstantsV2.VERT_SPIN_L2BACK);
            }
            else if (backTarget == BackTarget.RIGHT) {
                vertSpin.setPosition(MarvConstantsV2.VERT_SPIN_R2BACK);
            }

            if (drop) {
                automationState = AutomationState.DROP;
                dropTimer = System.currentTimeMillis();
            }
        }
        else if (automationState == AutomationState.DROP){

            if (System.currentTimeMillis() > dropTimer + MarvConstantsV2.VERT_SPIN_TODROP_MILLIS) {
                automationState = AutomationState.DOWN;
            }
        }


        lastHorizLiftLPosition = horizLiftLPosition;
    }

    long dropTimer;


}
