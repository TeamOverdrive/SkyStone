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
import org.firstinspires.ftc.teamcode.ftc2753.subsystems.Robot;
import org.firstinspires.ftc.teamcode.ftc2753.subsystems.Servos;
import org.firstinspires.ftc.teamcode.ftc2753.util.utilities;

import java.util.Locale;

@Config
@TeleOp(name = "Teleop", group = "TeleOp")
public class Teleop2 extends LinearOpMode {

    Robot robot = new Robot();

    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        init();

        Orientation angles;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if (gamepad1.y && (Math.abs(gamepad1.left_stick_y) > 0.05 || Math.abs(gamepad1.left_stick_y) > 0.05)) {
                robot.drive.teleDrive(angles,robot.drive.gyroAngle(1,90,angles), gamepad1);
            } else if (gamepad1.y) {
                gyroTurn(0.4,90,robot);
            } else {
                robot.drive.teleDrive(angles,gamepad1);
            }
            getBrake(robot);
            if (gamepad1.y) {
                initIMU();
            }
            robot.servos.setFoundationGrabber();
            robot.servos.setIntakeHeight();
            robot.drive.update();

        }
        requestOpModeStop();
    }
    public void getBrake(Robot robot) {
        if (gamepad1.left_bumper) {
            robot.drive.setBrake(robot.drive.motorFrontLeft);
            robot.drive.FrontLeft = 0;
            robot.drive.BackLeft = 0;
        }
        if (gamepad1.right_bumper) {
            robot.drive.setBrake(robot.drive.motorFrontRight);
            robot.drive.FrontRight = 0;
            robot.drive.BackRight = 0;
        }
        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            robot.drive.setBrake(robot.drive.motorBackLeft);
            robot.drive.setBrake(robot.drive.motorBackRight);
            robot.drive.kill();
        }
    }
    public void gyroTurn (double speed, double angle, Robot robot) {
        Orientation angles;
        // keep looping while we are still active, and not on heading.
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        while (opModeIsActive() && !robot.drive.onHeading(speed, angle, angles)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
    }
    public void initIMU() {

        BNO055IMU.Parameters IMUparameters = new BNO055IMU.Parameters();
        IMUparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMUparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUparameters.calibrationDataFile = "BNO055IMUCalibration.json";
        IMUparameters.loggingEnabled = true;
        IMUparameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUparameters);

    }

}



