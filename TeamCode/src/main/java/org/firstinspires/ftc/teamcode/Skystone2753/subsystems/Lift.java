package org.firstinspires.ftc.teamcode.Skystone2753.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.ExecutionException;

public class Lift extends Robot {

    public static final double COUNTS_PER_INCH = 77.28;
    public static final double GRAB_POS = 0.6;
    public static final double SUPER_GRAB_POS = 1;
    public static final double RELEASE_POS = 0.2;
    public static final double CAP_POS = 1.0;
    public static final double HOLD_POS = 0.7;


    Servo grabber, armLeft, armRight, capper;
    DcMotor liftLeft,liftRight;

    double liftLeftPower = 0, liftRightPower = 0;

    public Lift(LinearOpMode linearOpMode) {
        super.linearOpMode = linearOpMode;
        liftLeft = linearOpMode.hardwareMap.get(DcMotor.class, "lift_left");
        liftRight = linearOpMode.hardwareMap.get(DcMotor.class, "lift_right");
        grabber = linearOpMode.hardwareMap.get(Servo.class, "grabber");
        armLeft = linearOpMode.hardwareMap.get(Servo.class, "armservoleft");
        armRight = linearOpMode.hardwareMap.get(Servo.class, "armservoright");
        capper = linearOpMode.hardwareMap.get(Servo.class, "caprelease");
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void init() {
        liftLeft = super.linearOpMode.hardwareMap.get(DcMotor.class, "lift_left");
        liftRight = super.linearOpMode.hardwareMap.get(DcMotor.class, "lift_right");
        grabber = super.linearOpMode.hardwareMap.get(Servo.class, "grabber");
        armLeft = super.linearOpMode.hardwareMap.get(Servo.class, "armservoleft");
        armRight = super.linearOpMode.hardwareMap.get(Servo.class, "armservoright");
        capper = super.linearOpMode.hardwareMap.get(Servo.class, "caprelease");
    }
    public void up() {
        int lastPos = liftLeft.getCurrentPosition();
        while(liftLeft.getCurrentPosition() < lastPos + 4) {
            if (liftLeft.getCurrentPosition() < lastPos + 2) {
                setPower(0.8);
                lift();
            } else {
                setPower(0.4);
                lift();
            }

        }
    }
    public void up(int blockNum) {
        int lastPos = liftLeft.getCurrentPosition();
        while(liftLeft.getCurrentPosition() < lastPos + (4 * blockNum)) {
            if (liftLeft.getCurrentPosition() < lastPos + (4 * ((blockNum - 1) + 2))) {
                setPower(0.8);
                lift();
            } else {
                setPower(0.4);
                lift();
            }
        }
    }
    public void setPower(double speed) {
        liftLeftPower = speed;
        liftRightPower = -speed;
    }
    public void dilate(double dilation) {
        liftLeftPower *= dilation;
        liftRightPower *= dilation;
    }
    public void lift() {
        liftRight.setPower(liftRightPower);
        liftLeft.setPower(liftLeftPower);
    }
    public void run(double speed) {
        setPower(speed);
        lift();
    }
    public void grab() {
        grabber.setPosition(GRAB_POS);
    }
    public void release() {
        grabber.setPosition(RELEASE_POS);
    }
    public void rotate(double pos) {
        armRight.setPosition(pos);
        armLeft.setPosition(1-pos);
    }
    public void cap() {
        capper.setPosition(CAP_POS);
    }
    public void lock() {
        capper.setPosition(HOLD_POS);
    }
    public void superGrab() {
        grabber.setPosition(SUPER_GRAB_POS);
    }
}
