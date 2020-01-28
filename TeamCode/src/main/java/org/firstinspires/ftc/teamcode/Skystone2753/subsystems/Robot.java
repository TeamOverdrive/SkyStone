package org.firstinspires.ftc.teamcode.Skystone2753.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Skystone2753.vision.Vision;

public class Robot {

    public BNO055IMU imu;
    Orientation angles;

    public LinearOpMode linearOpMode = null;

    public driveTrain drive = null;

    public Vision vision = null;

    public FoundationGrabber foundation = null;

    public Lift lift = null;

    public Robot(LinearOpMode linearOpMode) {
        init(linearOpMode);
    }

    BNO055IMU.Parameters IMUparameters = new BNO055IMU.Parameters();

    public Robot() {}


    public void init(LinearOpMode linearOpMode) {

        this.linearOpMode = linearOpMode;
        initIMU();
        drive = new driveTrain(linearOpMode,imu);
        drive.initDrive();
        // vision = new Vision(linearOpMode);
        lift = new Lift(linearOpMode);
        foundation = new FoundationGrabber(linearOpMode);
        // lift.init();
        // vision.init();

    }
    public driveTrain getDrive() {
        return drive;
    }

    public void teleop(){
        double relativeAngle;
        relativeAngle = (Math.atan2(-linearOpMode.gamepad1.left_stick_y, linearOpMode.gamepad1.left_stick_x) - Math.PI / 4) - Math.toRadians(angles.firstAngle);
        if (Math.abs(relativeAngle) > Math.PI) {
            if (relativeAngle > 0)
                relativeAngle = -(Math.PI * 2 - Math.abs(relativeAngle));
            else if (relativeAngle > 0)
                relativeAngle = Math.PI * 2 - Math.abs(relativeAngle);
        }
        drive.setPower(relativeAngle, Math.sqrt(linearOpMode.gamepad1.left_stick_x * linearOpMode.gamepad1.left_stick_x +  linearOpMode.gamepad1.left_stick_y * linearOpMode.gamepad1.left_stick_y),
                linearOpMode.gamepad1.right_stick_x);
    }
    public void initIMU() {

        imu = linearOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        IMUparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMUparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUparameters.calibrationDataFile = "BNO055IMUCalibration.json";
        IMUparameters.loggingEnabled = true;
        IMUparameters.loggingTag = "IMU";
        imu.initialize(IMUparameters);
    }
}
