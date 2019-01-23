package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class MarvXICommon {

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    DcMotor hingeL;
    DcMotor hingeR;

    DcMotor expando;
    DcMotor lifto;

    CRServo intakeF;
    CRServo intakeR;

    DcMotor.ZeroPowerBehavior lastZeroPowerBehavior;

    public MarvXICommon(HardwareMap hardwareMap) {

        /*fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        this.setEncoderBehavior(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setEncoderBehavior(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);*/

        hingeL = hardwareMap.dcMotor.get("hingeL");
        hingeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hingeL.setDirection(DcMotorSimple.Direction.REVERSE);
        hingeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hingeR = hardwareMap.dcMotor.get("hingeR");
        hingeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        hingeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeF = hardwareMap.crservo.get("intakeF");
        intakeR = hardwareMap.crservo.get("intakeR");
        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void setHingeSpeed(double speed) {
        hingeL.setPower(speed);
        hingeR.setPower(speed);
    }

    public void setIntakeSpeed(double speed) {
        intakeF.setPower(speed);
        intakeR.setPower(speed);
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


}
