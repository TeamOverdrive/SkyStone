package org.firstinspires.ftc.teamcode.ftc2753.teleop;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ftc2753.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.ftc2753.subsystems.Servos;
import org.firstinspires.ftc.teamcode.ftc2753.util.utilities;

import java.util.Locale;

@Config
@TeleOp(name = "Teleop", group = "TeleOp")
public class Teleop2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DriveTrain drive = new DriveTrain();
        Servos servo = new Servos();
        servo.intakeLift.setPosition(1);

        waitForStart();

        drive.setBrake();

        Orientation angles;

        while (opModeIsActive() && !isStopRequested()) {

            angles = drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            drive.teleDrive(angles);
            getBrake(drive);
            if (gamepad1.y) {
                drive.initIMU();
            }
            drive.update();

        }
        requestOpModeStop();
    }
    public void getBrake(DriveTrain drive) {
        if (gamepad1.left_bumper) {
            drive.setBrake(drive.motorFrontLeft);
            drive.FrontLeft = 0;
            drive.BackLeft = 0;
        }
        if (gamepad1.right_bumper) {
            drive.setBrake(drive.motorFrontRight);
            drive.FrontRight = 0;
            drive.BackRight = 0;
        }
        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            drive.setBrake(drive.motorBackLeft);
            drive.setBrake(drive.motorBackRight);
            drive.kill();
        }
    }

}



