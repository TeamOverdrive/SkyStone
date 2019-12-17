package org.firstinspires.ftc.teamcode.Skystone2753;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "TESTING", group = "Does this even matter?")
public class SandboxAuto extends LinearOpMode {

    Robot robot = new Robot();
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);
        waitForStart();
        robot.getDrive().setUnit(robot.getDrive().INCHES);
        robot.getDrive().moveDist(2,0.1);
    }
    public void initMotors() {

        motorBackLeft = hardwareMap.get(DcMotor.class, "left_back");
        motorBackRight = hardwareMap.get(DcMotor.class, "right_back");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "left_front");
        motorFrontRight = hardwareMap.get(DcMotor.class, "right_front");

        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);

    }

}
