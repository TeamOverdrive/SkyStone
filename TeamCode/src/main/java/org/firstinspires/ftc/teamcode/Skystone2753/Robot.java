package org.firstinspires.ftc.teamcode.Skystone2753;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ftc2753.teleop.Sandbox;

public class Robot extends subsystems {

    BNO055IMU imu;
    Orientation angles;

    driveTrain drive;

    public Robot(SandboxAuto opMode) {

        init(opMode);

        drive = new driveTrain(opMode);

    }
    public Robot() {
    }
    public void init(SandboxAuto opMode) {

        BNO055IMU.Parameters IMUparameters = new BNO055IMU.Parameters();
        IMUparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMUparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUparameters.calibrationDataFile = "BNO055IMUCalibration.json";
        IMUparameters.loggingEnabled = true;
        IMUparameters.loggingTag = "IMU";

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUparameters);

    }
    public driveTrain getDrive() {
        return drive;
    }
}
