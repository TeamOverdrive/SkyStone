package org.firstinspires.ftc.teamcode.Skystone2753.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class FoundationGrabber extends Robot {
    Servo graber;
    public FoundationGrabber(LinearOpMode linearOpMode) {
        super.linearOpMode = linearOpMode;
        init();
    }
    public void init() {
        graber = linearOpMode.hardwareMap.get(Servo.class, "foundation");
    }
    public void up() {
        graber.setPosition(0.1);
    }
    public void down() {
        graber.setPosition(0);
    }
}
