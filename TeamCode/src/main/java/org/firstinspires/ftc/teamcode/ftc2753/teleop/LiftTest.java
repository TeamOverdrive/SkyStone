package org.firstinspires.ftc.teamcode.ftc2753.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Created by David Zheng | FTC 2753 Team Overdrive on 1/11/2020.
 */


@TeleOp
public class LiftTest extends LinearOpMode {


    private static final double RETRACTMULTIPLIER = 0.35;
    private DcMotor liftLeft, liftRight;

    private double liftPower;

    @Override
    public void runOpMode() throws InterruptedException {

        liftLeft = hardwareMap.get(DcMotorImplEx.class, "lift_left");
        liftRight = hardwareMap.get(DcMotorImplEx.class, "lift_right");

        liftLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            liftPower = -gamepad1.right_stick_y;
            if(liftPower <= 0)
                liftPower = liftPower*RETRACTMULTIPLIER;
            liftLeft.setPower(liftPower);
            liftRight.setPower(liftPower);

            telemetry.addData("Lift Power", liftPower);
            telemetry.update();

        }
    }
}
