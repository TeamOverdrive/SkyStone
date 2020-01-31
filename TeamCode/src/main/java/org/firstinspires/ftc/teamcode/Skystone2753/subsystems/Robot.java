// Made by Timothy *before qualification*
package org.firstinspires.ftc.teamcode.Skystone2753.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Skystone2753.vision.Vision;

// Instantiate Robot class to gain access to all necessary basic methods
public class Robot {

    double x;
    double y;

    public BNO055IMU imu;
    Orientation angles;

    // LinearOpMode required for telemetry and hardwareMap
    public LinearOpMode linearOpMode = null;

    //  all subsystems extend robot in order to gain access to shared code such as imu
    // NOTE: super of subsystems is NOT set when instantiated and is not the same as the calling Robot
    public driveTrain drive = null;
    public Lift lift = null;

    BNO055IMU.Parameters IMUparameters = new BNO055IMU.Parameters();

    // Robot must have LinearOpMode for hardwareMap and telemetry
    public Robot(LinearOpMode linearOpMode) {
        init(linearOpMode);
    }

    public void init(LinearOpMode linearOpMode) {

        this.linearOpMode = linearOpMode;
        initIMU();
        drive = new driveTrain(linearOpMode,imu);
        drive.initDrive();
        lift = new Lift(linearOpMode);

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
