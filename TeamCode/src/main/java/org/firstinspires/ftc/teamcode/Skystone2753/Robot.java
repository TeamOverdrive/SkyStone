package org.firstinspires.ftc.teamcode.Skystone2753;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ftc2753.teleop.Sandbox;

public class Robot {

    public BNO055IMU imu;
    Orientation angles;

    public LinearOpMode linearOpMode = null;

    public driveTrain drive = null;

    public Robot(LinearOpMode linearOpMode) {
        init(linearOpMode);
    }
    public Robot() {}

    public void init(LinearOpMode linearOpMode) {

        this.linearOpMode = linearOpMode;
        BNO055IMU.Parameters IMUparameters = new BNO055IMU.Parameters();
        IMUparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMUparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUparameters.calibrationDataFile = "BNO055IMUCalibration.json";
        IMUparameters.loggingEnabled = true;
        IMUparameters.loggingTag = "IMU";

        imu = linearOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUparameters);
        drive = new driveTrain(linearOpMode,imu);
        drive.initDrive();

    }
    public driveTrain getDrive() {
        return drive;
    }
}
