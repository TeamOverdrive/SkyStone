package org.firstinspires.ftc.teamcode.ftc2753.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Servos extends Robot {

    public final static double SIDE_UP = 180/270;
    public final static double SIDE_DOWN = 0/270;

    public Servo sideGrabber, foundationLeft, foundationRight,intakeLift;

    public Servos() {

    }
    public void init(){
        sideGrabber = hardwareMap.get(ServoImplEx.class, "sideGrabber");
        foundationLeft = hardwareMap.get(ServoImplEx.class, "foundationLeft");
        foundationRight = hardwareMap.get(ServoImplEx.class, "foundationRight");
        intakeLift = hardwareMap.get(ServoImplEx.class, "liftIntake");

        releaseFoundation();
        sideGrabber.setPosition(0.5f);


    }
    public void setIntakeHeight() {
        if (gamepad2.y) {
            intakeLift.setPosition(0.6);
        }
        if (gamepad2.a) {
            intakeLift.setPosition(0);
        }
    }
    public void grabFoundation() {
        foundationRight.setPosition(1.0f);
        foundationLeft.setPosition(0.0f);
    }

    public void releaseFoundation(){
        foundationRight.setPosition(0.5f);
        foundationLeft.setPosition(0.5f);
    }
    public void setFoundationGrabber() {
        if(gamepad2.b)
            grabFoundation();
        if(gamepad2.x)
            releaseFoundation();
    }

}
