package org.firstinspires.ftc.teamcode.ftc2753.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

public class Robot {

    //public Telemetry telemetry = new TelemetryImpl(this);


    public BNO055IMU imu;

    public Servos servos = new Servos();
    public Intake intake = new Intake();
    public ExtendedDriveTrain drive = new ExtendedDriveTrain();

    public Robot(LinearOpMode opMode) {

        drive.init(opMode);
        intake.init();
        servos.init();

    }


}
