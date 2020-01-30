// Last changed by Timothy 1/29/2020
package org.firstinspires.ftc.teamcode.Skystone2753.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.ExecutionException;

// contains all methods for raising and lowering lift subsystem. Includes v4b, grabber, capper.
public class Lift extends Robot {

    public static final double COUNTS_PER_INCH = 77.28;

    // preset positions use if applicable
    public static final double GRAB_POS = 0.6;
    // super grab position used for grabbing blocks while facing width-wise
    public static final double SUPER_GRAB_POS = 1;
    public static final double RELEASE_POS = 0.2;
    public static final double CAP_POS = 1.0;
    public static final double HOLD_POS = 0.7;

    // armLeft and armRight are v4b servos
    Servo grabber, armLeft, armRight, capper;

    DcMotor liftLeft,liftRight;

    // power variables seperate in order to preform mathmatical functions before setting power
    double liftLeftPower = 0, liftRightPower = 0;

    // requires LinearOpMode to get hardwareMap and telemetry
    public Lift(LinearOpMode linearOpMode) {

        super.linearOpMode = linearOpMode;

        init();

        brake();

    }
    // do not use init() unless lift is down
    public void init() {
        liftLeft = super.linearOpMode.hardwareMap.get(DcMotor.class, "lift_left");
        liftRight = super.linearOpMode.hardwareMap.get(DcMotor.class, "lift_right");
        grabber = super.linearOpMode.hardwareMap.get(Servo.class, "grabber");
        armLeft = super.linearOpMode.hardwareMap.get(Servo.class, "armservoleft");
        armRight = super.linearOpMode.hardwareMap.get(Servo.class, "armservoright");
        capper = super.linearOpMode.hardwareMap.get(Servo.class, "caprelease");
    }
    public void brake() {
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void removeBrake() {
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    // sets a power but does not run lift until lift() is called
    public void inputSpeed(double speed) {
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
    public void setPower(double speed) {
        inputSpeed(speed);
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
