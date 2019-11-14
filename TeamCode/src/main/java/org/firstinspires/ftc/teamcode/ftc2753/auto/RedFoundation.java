package org.firstinspires.ftc.teamcode.ftc2753.auto;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ftc2753.subsystems.AutoDriveTrain;
import org.firstinspires.ftc.teamcode.ftc2753.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.ftc2753.subsystems.Robot;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;


import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;

@Autonomous(name="Red Foundation", group="auto")

public class RedFoundation extends LinearOpMode {

    Orientation angles;

    Robot robot = new Robot();

    NormalizedColorSensor colorSensor;
    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need something analogous when you
     * use a color sensor on your robot */
    View relativeLayout;

    public void runOpMode() throws InterruptedException {

        robot.servos.releaseFoundation();

        // Wait for the start button to be pressed.
        waitForStart();

        robot.drive.move(-20,0.2f,4);
        update();

        robot.drive.strafe("LEFT",10,0.5,7);
        update();

        robot.drive.move(-14,0.2f,4);
        update();

        robot.drive.move(0);
        update();

        robot.servos.grabFoundation();

        pause(1);

        robot.drive.move(30,0.6f,10);
        update();

        robot.drive.strafe("LEFT",8,0.6f,3);

        gyroTurn(0.4f,90);

        robot.drive.move(-6,0.4f,2);
        update();

        robot.drive.strafe("LEFT", 12,0.6f,2);

        robot.drive.kill();

        pause(0.5f);

        robot.servos.releaseFoundation();

        robot.drive.strafe("LEFT",12,0.6f,3);

        pause(15);

        robot.drive.move(34,1,3);
        update();

    }

    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        while (opModeIsActive() && !robot.drive.onHeading(speed, angle, angles)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
    }
    public void update() {
        robot.drive.motorFrontRight.setPower(Math.abs(robot.drive.speed));
        robot.drive.motorBackRight.setPower(Math.abs(robot.drive.speed));
        robot.drive.motorFrontLeft.setPower(Math.abs(robot.drive.speed));
        robot.drive.motorBackLeft.setPower(Math.abs(robot.drive.speed));

        ElapsedTime runtime = new ElapsedTime();

        while (opModeIsActive() &&
                (runtime.seconds() < robot.drive.timeout) &&
                (robot.drive.motorFrontRight.isBusy() && robot.drive.motorBackRight.isBusy() && robot.drive.motorFrontLeft.isBusy() && robot.drive.motorBackLeft.isBusy())) {

        }

        robot.drive.motorFrontRight.setPower(0);
        robot.drive.motorBackRight.setPower(0);
        robot.drive.motorFrontLeft.setPower(0);
        robot.drive.motorBackLeft.setPower(0);
    }
    public void pause(float seconds) {
        ElapsedTime runtime = new ElapsedTime();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {}
    }
}
