package org.firstinspires.ftc.teamcode.ftc2753.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Created by David Zheng | FTC 2753 Team Overdrive on 1/11/2020.
 */


@TeleOp
public class ServoTester extends LinearOpMode {

    private boolean isDL, wasDL = false;
    private boolean isDR, wasDR = false;

    private Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Plug in servo to ");
        telemetry.addLine("Expansion Hub Servo Port 0");
        telemetry.addLine("before running this opmode");

        servo = hardwareMap.get(ServoImplEx.class, "servo_test");

        waitForStart();
        telemetry.clearAll();
        servo.setPosition(0.5);
        while (opModeIsActive() && !isStopRequested()) {


            if(Math.abs(gamepad1.left_stick_y) < 0.05)  {
                if (gamepad1.dpad_up)
                    servo.setPosition(1);
                if (gamepad1.dpad_down)
                    servo.setPosition(0);
                if ((isDR = gamepad1.dpad_right) && !wasDR)
                    servo.setPosition(servo.getPosition() + 0.05);
                if ((isDL = gamepad1.dpad_left) && !wasDL)
                    servo.setPosition(servo.getPosition() - 0.05);
                if (gamepad1.x)
                    servo.setPosition(0.5);
                if (gamepad1.y)
                    servo.setPosition(0.75);
                if (gamepad1.a)
                    servo.setPosition(0.25);
            }
            else
            {
                servo.setPosition(((-gamepad1.left_stick_y)/2)+0.5);
            }

            wasDL = isDL;
            wasDR = isDR;

            //(isRB = gamepad1.right_bumper) && !wasRB)

            telemetry.addData("Servo PWM", servo.getPosition());
            telemetry.update();
        }
    }
}
