package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MarvXICommon {

    public static int HINGE_THRESH = 999000;

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    DcMotor hingeL;
    DcMotor hingeR;

    DcMotor expand;
    DcMotor lift;

    CRServo intakeF;
    CRServo intakeR;

    Servo rotate;

    DcMotor.ZeroPowerBehavior lastZeroPowerBehavior;

    Rev2mDistanceSensor liftDist;

    BNO055IMU imu;


    public static int AP_TICKS_PER_INCH = 0;
    public static int AP_COUNTS_TO_STABLE = 0;
    public static double NAV_GAIN_PER_INCH = 0;
    public static double ORIENT_GAIN_PER_INCH = 0;

    public MarvXICommon(HardwareMap hardwareMap) {

        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        liftDist = hardwareMap.get(Rev2mDistanceSensor.class, "dist");

        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        this.setEncoderBehavior(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setEncoderBehavior(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hingeL = hardwareMap.dcMotor.get("hingeL");
        hingeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        hingeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hingeR = hardwareMap.dcMotor.get("hingeR");
        hingeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hingeR.setDirection(DcMotorSimple.Direction.REVERSE);
        hingeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeF = hardwareMap.crservo.get("intakeF");
        intakeR = hardwareMap.crservo.get("intakeR");
        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);

        lift = hardwareMap.dcMotor.get("lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rotate = hardwareMap.servo.get("rotate");
        rotate.setDirection(Servo.Direction.REVERSE);
        setServoExtendedRange(rotate, 500, 2500);
        rotate.setPosition(0);

        expand = hardwareMap.dcMotor.get("expand");
        expand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        expand.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void stop() {
        rotate.getController().pwmDisable();
    }

    public boolean hingeIsInDrop() {
        return (hingeL.getCurrentPosition() + hingeR.getCurrentPosition()) / 2.0 < HINGE_THRESH;
    }

    public void setExpandDefaultMode() {
        expand.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setExpandHoldMode() {
        expand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        expand.setTargetPosition(expand.getCurrentPosition());
        expand.setPower(0.25);
    }

    public void setHingeDefaultMode() {
        hingeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hingeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setHingeHoldMode() {
        hingeR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hingeR.setTargetPosition(hingeR.getCurrentPosition());
        hingeR.setPower(1);
        hingeL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hingeL.setTargetPosition(hingeL.getCurrentPosition());
        hingeL.setPower(1);
    }

    public void setLiftDefaultMode() {
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setLiftHoldMode() {
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(lift.getCurrentPosition());
        lift.setPower(0.25);
    }

    public void setRotateStraight() {
        rotate.setPosition(4.0/5.0);
    }

    public void setRotateIn() {
        rotate.setPosition(0);
    }

    public void setRotateL() {
        rotate.setPosition(3.0/5.0);
    }

    public void setRotateR() {
        rotate.setPosition(1);
    }

    public void telemPoses(Telemetry telemetry) {
        telemetry.addData("hingeL", hingeL.getCurrentPosition());
        telemetry.addData("hingeR", hingeR.getCurrentPosition());
        telemetry.addData("lift", lift.getCurrentPosition());
        telemetry.addData("expand", expand.getCurrentPosition());
        telemetry.addData("dist", liftDist.getDistance(DistanceUnit.INCH));
    }

    public void setServoExtendedRange(Servo servo, int min, int max) {
        ServoControllerEx controller = (ServoControllerEx) servo.getController();
        int servoPort = servo.getPortNumber();
        PwmControl.PwmRange range = new PwmControl.PwmRange(min, max);
        controller.setServoPwmRange(servoPort, range);
    }

    public void setHingeSpeed(double speed) {
        hingeL.setPower(speed);
        hingeR.setPower(speed);
    }

    public void setIntakeSpeed(double speed) {
        intakeF.setPower(speed);
        intakeR.setPower(speed);
    }

    public void setLiftSpeed(double speed) {
        lift.setPower(speed);
    }

    public void setExpandSpeed(double speed) {
        expand.setPower(speed);
    }

    public double pos2vel(double P, double posTarg, double posCur, double Vmax) {
        double vel = P * (posTarg - posCur);
        if (vel > 0) {
            vel = Math.min(Vmax, vel);
        }
        else {
            vel = Math.max(-Vmax, vel);
        }
        return vel;
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
        return fr.getCurrentPosition();
    }
    public double getQuadPacerY() {
        return br.getCurrentPosition();
    }

    public DcMotor getQuadPacerMotorX() {
        return fr;
    }
    public DcMotor getQuadPacerMotorY() {
        return br;
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


}
