package org.firstinspires.ftc.teamcode.Skystone2753;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ftc2753.subsystems.DriveTrain;

public class Robot extends subsystems {

    BNO055IMU imu;
    Orientation angles;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    driveTrain drive;

    public Robot(LinearOpMode opMode) {

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        init(opMode);

        drive = new driveTrain(opMode);

    }
    public Robot() {
    }
    public void init(LinearOpMode opMode) {

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(this.parameters);

    }
    public driveTrain getDrive() {
        return drive;
    }
}
