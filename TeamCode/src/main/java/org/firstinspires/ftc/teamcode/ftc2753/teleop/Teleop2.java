package org.firstinspires.ftc.teamcode.ftc2753.teleop;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ftc2753.subsystems.DriveTrain;

import java.util.Locale;

@Config
@TeleOp(name = "Teleop2.1", group = "TeleOp")
public class Teleop2 extends LinearOpMode {

    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor intake;

    private Servo sideGrabber;
    private Servo sensorRotator;
    private Servo foundationLeft;
    private Servo foundationRight;

    private float intakeSpeed;

    /*
    note that with the config annotation above public class Teleop2,
    the below public static non-final variables can be edited on the fly with FTC Dashboard
    */
    //sideUp and sideDown are used for both the stone side grabber and the sensor array actuator
    public static double sideUp = 180/270;
    public static double sideDown = 0/270;
    public static double grabberDiagnostic = 0.5; //grabber is in continuous mode rn

    //values for the foundation grabber servos; note foundationGrab() and foundationRelease() methods
    public static double foundationUp = 1;
    public static double foundationDown = 0;
    public static double foundationDiagnostic = 0.5;

    DriveTrain drive = new DriveTrain();


    @Override
    public void runOpMode() throws InterruptedException {

        BNO055IMU imu;
        Orientation angles;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        initMotors();
        initServos();

        waitForStart();



        while (opModeIsActive() && !isStopRequested()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            teleDrive(angles);
            if (gamepad1.left_bumper) {
                setBrake(motorFrontLeft);
                drive.FrontLeft = 0;
                drive.BackLeft = 0;
            }
            if (gamepad1.right_bumper) {
                setBrake(motorFrontRight);
                drive.FrontRight = 0;
                drive.BackRight = 0;
            }
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                setBrake(motorBackLeft);
                setBrake(motorBackRight);
                stopMove();
            }

            if (gamepad2.right_trigger > 0.2)
            {
                intakeSpeed = gamepad2.right_trigger * 1.25f;
            }
            else if (gamepad2.left_trigger > 0.2)
            {
                intakeSpeed = gamepad2.left_trigger * -1.25f;
            }
            else
                intakeSpeed = 0;

            intake.setPower(intakeSpeed);

            drive.BackLeft += gamepad1.left_trigger;
            drive.FrontLeft += gamepad1.left_trigger;
            drive.BackRight += gamepad1.right_trigger;
            drive.FrontRight += gamepad1.right_trigger;

            //Servos
            if(gamepad2.b){
                sensorRotator.setPosition(sideDown);
            }

            else{
                sensorRotator.setPosition(sideUp);
            }

            sideGrabber.setPosition(grabberDiagnostic);
            foundationLeft.setPosition(foundationDiagnostic);

            if(gamepad2.a)
                grabFoundation();
            if(gamepad2.y)
                releaseFoundation();


            removeBrake();
            update();

            //Telemetry
            telemetry.addData("Heading (Z?)", formatAngle(angles.angleUnit, angles.firstAngle));
            telemetry.addData("back left ",drive.BackLeft);
            telemetry.addData("front left ",drive.FrontLeft);
            telemetry.addData("front right ",drive.FrontRight);
            telemetry.addData("back right ",drive.BackRight);
            telemetry.update();
        }
        requestOpModeStop();
    }

    private void grabFoundation() {
        foundationRight.setPosition(foundationDown);
        foundationLeft.setPosition(1-foundationDown);
    }

    private void releaseFoundation(){
        foundationRight.setPosition(foundationUp);
        foundationLeft.setPosition(1-foundationUp);
    }

    public void initMotors() {

        motorBackLeft = hardwareMap.get(DcMotor.class, "left_back");
        motorBackRight = hardwareMap.get(DcMotor.class, "right_back");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "left_front");
        motorFrontRight = hardwareMap.get(DcMotor.class, "right_front");
        intake = hardwareMap.get(DcMotor.class, "intake");

        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);

    }

    public void initServos(){
        sideGrabber = hardwareMap.get(ServoImplEx.class, "sideGrabber");
        sensorRotator = hardwareMap.get(ServoImplEx.class, "sensor");
        foundationLeft = hardwareMap.get(ServoImplEx.class, "foundationLeft");
        foundationRight = hardwareMap.get(ServoImplEx.class, "foundationRight");
    }

    public void teleDrive(Orientation angles) {
        double relativeAngle;
        relativeAngle = (Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4) - Math.toRadians(angles.firstAngle);
        if (Math.abs(relativeAngle) > Math.PI) {
            if (relativeAngle > 0)
                relativeAngle = -(Math.PI * 2 - Math.abs(relativeAngle));
            else if (relativeAngle > 0)
                relativeAngle = Math.PI * 2 - Math.abs(relativeAngle);
        }

        drive.move(relativeAngle, Math.sqrt(gamepad1.left_stick_x * gamepad1.left_stick_x +  gamepad1.left_stick_y * gamepad1.left_stick_y),
                -gamepad1.right_stick_x);
    }
    public void update() {

        motorFrontLeft.setPower(drive.FrontLeft);
        motorFrontRight.setPower(drive.FrontRight);
        motorBackLeft.setPower(drive.BackLeft);
        motorBackRight.setPower(drive.BackRight);

    }
    public void setBrake(DcMotor motor) {

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void removeBrake() {

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
    public void stopMove() {

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }

    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}



