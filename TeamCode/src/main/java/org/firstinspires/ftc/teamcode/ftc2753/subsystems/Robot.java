package org.firstinspires.ftc.teamcode.ftc2753.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

public class Robot {

    public DistanceSensor distSensor;

    public Gamepad gamepad1 = null;
    public Gamepad gamepad2 = null;
    //public Telemetry telemetry = new TelemetryImpl(this);


    public BNO055IMU imu;

    public HardwareMap hardwareMap = null;

    public Servos servos = new Servos();
    public Intake intake = new Intake();
    public ExtendedDriveTrain drive = new ExtendedDriveTrain();

    public Robot() {

        drive.init();
        intake.init();
        servos.init();
        initIMU();
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
