package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
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

    CRServo horizSpinL;
    CRServo horizSpinR;
    Servo horizLiftL;
    Servo horizLiftR;

    double lastHorizLiftLPosition = -1;

    Servo vertLiftL;
    Servo vertLiftR;
    Servo vertSpin;

    Servo vertSwing;
    Servo vertLatch;

    ColorSensor color1;
    DistanceSensor distance1;
    ColorSensor color2;
    DistanceSensor distance2;

    Rev2mDistanceSensor altitude;

    public enum BackTarget {RIGHT, LEFT}
    public enum DropTarget {FARTHER, LESSER, NEUTRAL}
    public enum SwingTarget {CENTER, LEFT, RIGHT}
    public enum LiftTarget {FARTHER, LESSER}
    public enum AutomationState {CLEAR_READY, CLEARING, RELEASE_READY, RELEASING};

    BackTarget backTarget = null;
    DropTarget dropTarget = null;
    SwingTarget swingTarget = null;
    LiftTarget liftTarget = null;
    public AutomationState automationState = AutomationState.CLEAR_READY;
    public AutomationState lastAutomationState = null;

    DcMotor.ZeroPowerBehavior lastZeroPowerBehavior;

    public MarvXCommonV2(HardwareMap hardwareMap, boolean maintainExpandoHoriz) {

        color1 = hardwareMap.get(ColorSensor.class, "cd1");
        distance1 = hardwareMap.get(DistanceSensor.class, "cd1");
        color2 = hardwareMap.get(ColorSensor.class, "cd2");
        distance2 = hardwareMap.get(DistanceSensor.class, "cd2");

        altitude = hardwareMap.get(Rev2mDistanceSensor.class, "dist");

        fl = hardwareMap.dcMotor.get("fl");

        fr = hardwareMap.dcMotor.get("fr");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);

        bl = hardwareMap.dcMotor.get("bl");

        br = hardwareMap.dcMotor.get("br");
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        this.setEncoderBehavior(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setEncoderBehavior(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        expandoHorizL = hardwareMap.dcMotor.get("expandoHoriz");
        expandoHorizL.setDirection(DcMotorSimple.Direction.REVERSE);
        expandoHorizL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //if (!maintainExpandoHoriz) {expandoHorizL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
        expandoHorizL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        expandoHorizL.setPower(0);

        /*expandoHorizR = hardwareMap.dcMotor.get("expandoHorizR");
        expandoHorizR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (!maintainExpandoHoriz) {expandoHorizR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
        expandoHorizR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
*/
        expandoVertL = hardwareMap.dcMotor.get("expandoVertL");
        expandoVertL.setDirection(DcMotorSimple.Direction.REVERSE);
        expandoVertL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //if (!maintainExpandoHoriz) {expandoVertL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
        expandoVertL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        expandoVertL.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_DOWN);
        //if (!maintainExpandoHoriz) {expandoVertL.setPower(1);}

        expandoVertR = hardwareMap.dcMotor.get("expandoVertR");
        expandoVertR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //if (!maintainExpandoHoriz) {expandoVertR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
        expandoVertR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        expandoVertR.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_DOWN);
        //if (!maintainExpandoHoriz) {expandoVertR.setPower(1);}

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

        vertSwing = hardwareMap.servo.get("vertSwing");
        vertSwing.setPosition(MarvConstantsV2.VERT_SWING_CENTER);

        vertLatch = hardwareMap.servo.get("vertLatch");

        horizSpinL = hardwareMap.crservo.get("horizSpinL");
        horizSpinR = hardwareMap.crservo.get("horizSpinR");
        horizSpinR.setDirection(DcMotorSimple.Direction.REVERSE);

        /*imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu.initialize(parameters);*/
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
        return br.getCurrentPosition();
    }
    public double getQuadPacerY() {
        return fr.getCurrentPosition();
    }

    public DcMotor getQuadPacerMotorX() {
        return br;
    }
    public DcMotor getQuadPacerMotorY() {
        return fr;
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


    public void runAutomation(boolean drop, boolean fstart) {
        double horizLiftLPosition = horizLiftL.getPosition();


        if (automationState == AutomationState.CLEAR_READY) {
            if ((horizLiftL.getPosition() == MarvConstantsV2.HORIZ_LIFT_UP_NEUTRAL && lastHorizLiftLPosition == MarvConstantsV2.HORIZ_LIFT_UP_DUMPING) || fstart) {

                if (distance1.getDistance(DistanceUnit.MM) < 55 || distance2.getDistance(DistanceUnit.MM) < 55 || fstart) {
                    automationState = AutomationState.CLEARING;
                }

                double lWarmScore = (float)color1.red() / color1.blue();
                double rWarmScore = (float)color2.red() / color2.blue();

                if (lWarmScore < 1.5 && rWarmScore < 1.5) {
                    backTarget = BackTarget.LEFT;
                    dropTarget = DropTarget.NEUTRAL;
                    swingTarget = SwingTarget.CENTER;
                    liftTarget = LiftTarget.LESSER;
                }
                else if (lWarmScore > 1.5 && rWarmScore > 1.5) {
                    backTarget = BackTarget.LEFT;
                    //dropTarget = DropTarget.LESSER; // CHANGED FOR SWING
                    dropTarget = DropTarget.NEUTRAL;
                    swingTarget = SwingTarget.LEFT;
                    liftTarget = LiftTarget.FARTHER;
                }
                else if (lWarmScore > 1.5 && rWarmScore < 1.5){
                    // Cube on left
                    backTarget = BackTarget.RIGHT;
                    dropTarget = DropTarget.LESSER;
                    swingTarget = SwingTarget.LEFT;
                    liftTarget = LiftTarget.FARTHER;
                }
                else if (lWarmScore < 1.5 && rWarmScore > 1.5) {
                    // Cube on right
                    backTarget = BackTarget.LEFT;
                    dropTarget = DropTarget.FARTHER;
                    swingTarget = SwingTarget.LEFT;
                    liftTarget = liftTarget.FARTHER;
                }

            }
        }
        else if (automationState == AutomationState.CLEARING) {
            if (expandoVertL.getCurrentPosition() > MarvConstantsV2.EXPANDO_VERT_BOXFREE || expandoVertR.getCurrentPosition() > MarvConstantsV2.EXPANDO_VERT_BOXFREE) {
                automationState = AutomationState.RELEASE_READY;
                minusGTimer = System.currentTimeMillis();
                swingTimer = System.currentTimeMillis();
            }
        }
        else if (automationState == AutomationState.RELEASE_READY) {
            if (System.currentTimeMillis() < minusGTimer + MarvConstantsV2.VERT_LIFT_TOMINUSG_MILLIS) {
                vertLiftL.setPosition(MarvConstantsV2.VERT_LIFT_MINUSG);
                vertLiftR.setPosition(MarvConstantsV2.VERT_LIFT_MINUSG);
            }
            else {
                if (liftTarget != LiftTarget.LESSER) {
                    vertLiftL.setPosition(MarvConstantsV2.VERT_LIFT_UP);
                    vertLiftR.setPosition(MarvConstantsV2.VERT_LIFT_UP);
                }
            }

            if (System.currentTimeMillis() > swingTimer + MarvConstantsV2.VERT_LIFT_TOSWING_MILLIS) {
                if (swingTarget == SwingTarget.CENTER) {
                    vertSwing.setPosition(MarvConstantsV2.VERT_SWING_CENTER);
                }
                else if (swingTarget == SwingTarget.LEFT) {
                    vertSwing.setPosition(MarvConstantsV2.VERT_SWING_LEFT);
                }
                else if (swingTarget == SwingTarget.RIGHT) {
                    vertSwing.setPosition(MarvConstantsV2.VERT_SWING_RIGHT);
                }
            }

            if (drop) {
                automationState = AutomationState.RELEASING;
                dropTimer = System.currentTimeMillis();
            }
        }
        else if (automationState == AutomationState.RELEASING){
            if (System.currentTimeMillis() > dropTimer + MarvConstantsV2.VERT_SPIN_TODROP_MILLIS) {
                automationState = AutomationState.CLEAR_READY;
            }
        }



        if (automationState == AutomationState.CLEAR_READY && lastAutomationState != AutomationState.CLEAR_READY) {
            expandoVertL.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_DOWN);
            expandoVertR.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_DOWN);
            expandoVertL.setPower(MarvConstantsV2.EXPANDO_VERT_TODOWN_SPEED);
            expandoVertR.setPower(MarvConstantsV2.EXPANDO_VERT_TODOWN_SPEED);
            vertLiftL.setPosition(MarvConstantsV2.VERT_LIFT_DOWN);
            vertLiftR.setPosition(MarvConstantsV2.VERT_LIFT_DOWN);
            vertSpin.setPosition(MarvConstantsV2.VERT_SPIN_NEUTRAL);
            vertSwing.setPosition(MarvConstantsV2.VERT_SWING_CENTER);
        }
        else if (automationState == AutomationState.CLEARING && lastAutomationState != AutomationState.CLEARING) {
            if (liftTarget == LiftTarget.FARTHER) {
                expandoVertL.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_UP);
                expandoVertR.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_UP);
            }
            else {
                expandoVertL.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_SAFE);
                expandoVertR.setTargetPosition(MarvConstantsV2.EXPANDO_VERT_SAFE);
            }
            expandoVertL.setPower(MarvConstantsV2.EXPANDO_VERT_TOUP_SPEED);
            expandoVertR.setPower(MarvConstantsV2.EXPANDO_VERT_TOUP_SPEED);
        }
        else if (automationState == AutomationState.RELEASE_READY && lastAutomationState != AutomationState.RELEASE_READY) {
            if (backTarget == BackTarget.LEFT) {
                vertSpin.setPosition(MarvConstantsV2.VERT_SPIN_L2BACK);
            }
            else if (backTarget == BackTarget.RIGHT) {
                vertSpin.setPosition(MarvConstantsV2.VERT_SPIN_R2BACK);
            }
        }
        else if (automationState == AutomationState.RELEASING && lastAutomationState != AutomationState.RELEASING) {
            if (backTarget == BackTarget.LEFT) {
                if (dropTarget == DropTarget.FARTHER) {vertSpin.setPosition(MarvConstantsV2.VERT_SPIN_L2BACK_FARTHER);}
                else if (dropTarget == DropTarget.LESSER) {vertSpin.setPosition(MarvConstantsV2.VERT_SPIN_L2BACK_LESSER);}
            }
            else if (backTarget == BackTarget.RIGHT) {
                if (dropTarget == DropTarget.FARTHER) {vertSpin.setPosition(MarvConstantsV2.VERT_SPIN_R2BACK_FARTHER);}
                else if (dropTarget == DropTarget.LESSER) {vertSpin.setPosition(MarvConstantsV2.VERT_SPIN_R2BACK_LESSER);}
            }

            if (dropTarget == DropTarget.NEUTRAL) {
                vertSpin.setPosition(MarvConstantsV2.VERT_SPIN_NEUTRAL);
            }
        }


        lastHorizLiftLPosition = horizLiftLPosition;
        lastAutomationState = automationState;
    }

    long dropTimer;
    long minusGTimer;
    long swingTimer;


}
