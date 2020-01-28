package org.firstinspires.ftc.teamcode.Skystone2753;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class intake extends Robot {
    public double intakeSpeed = 0;
    public final double INTAKE = 1;
    public final double OUTTAKE = -1;
    public final double STOP = 0;
    public final double UPPOS = 0;
    public final double DOWNPOS = 0;
    public final double MIDPOS = 0.6;
    public DcMotor intake;
    public Servo intakeLift;
    public intake(Team2753LinearOpMode linearOpMode) {
        super.linearOpMode = linearOpMode;
        init();
    }
    private void init() {
        intake = linearOpMode.hardwareMap.get(DcMotor.class, "intake");
        intakeLift = linearOpMode.hardwareMap.get(Servo.class, "liftIntake");
    }
    public void intake(){
        intakeSpeed = INTAKE;
        run();
    }
    public void outtake() {
        intakeSpeed = OUTTAKE;
        run();
    }
    public void pause() {
        intakeSpeed = STOP;
        run();
    }
    public void run() {
        intake.setPower(intakeSpeed);
    }
    public void setSpeed(double speed) {
        intakeSpeed = speed;
    }
    public void up() {
        intakeLift.setPosition(UPPOS);
    }
    public void down() {
        intakeLift.setPosition(DOWNPOS);
    }
    public void rest() {
        intakeLift.setPosition(MIDPOS);
    }
    public void setPos(double pos) {
        intakeLift.setPosition(pos);
    }
}
