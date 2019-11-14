package org.firstinspires.ftc.teamcode.ftc2753.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake extends Robot {

    private float intakeSpeed;
    DcMotor intake;

    public Intake() {

    }
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        servos.intakeLift.setPosition(1);
    }
    public void setSpeed() {
        if (gamepad2.right_trigger > 0.2)
            intakeSpeed = gamepad2.right_trigger * 1.25f;
        else if (gamepad2.left_trigger > 0.2)
            intakeSpeed = gamepad2.left_trigger * -1.25f;
        else
            intakeSpeed = 0;
        intake.setPower(intakeSpeed);

    }
    public void setSpeed(double speed) {
        intake.setPower(speed);
    }

}
