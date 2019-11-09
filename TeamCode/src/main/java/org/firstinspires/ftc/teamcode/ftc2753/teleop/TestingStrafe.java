package org.firstinspires.ftc.teamcode.ftc2753.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test Me", group = "tests")
public class TestingStrafe extends LinearOpMode {

    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;

    private Servo sideGrabber;

    public void runOpMode() {

        initMotors();

        waitForStart();

        sideGrabber = hardwareMap.get(ServoImplEx.class, "sideGrabber");

        ElapsedTime runtime = new ElapsedTime();

        while(opModeIsActive()) {
            if (gamepad1.a) {
                sideGrabber.setPosition(0.5f);
            }
            if (gamepad1.b) {
                sideGrabber.setPosition(1.0f);
            }
            if (gamepad1.y) {
                sideGrabber.setPosition(0.0f);
            }
            if (gamepad1.x) {
                sideGrabber.setPosition(0.33f);
            }
            //motorBackLeft.setPower(0.2 * 0.87);
            //motorFrontLeft.setPower(-0.2);
            //motorBackRight.setPower(-0.2 * 0.87);
            //motorFrontRight.setPower(0.2);
            // if (runtime.seconds() > 1)
                // requestOpModeStop();

        }

    }
    public void initMotors() {

        motorBackLeft = hardwareMap.get(DcMotor.class, "left_back");
        motorBackRight = hardwareMap.get(DcMotor.class, "right_back");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "left_front");
        motorFrontRight = hardwareMap.get(DcMotor.class, "right_front");

        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
