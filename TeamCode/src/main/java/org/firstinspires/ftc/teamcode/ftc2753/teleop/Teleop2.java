package org.firstinspires.ftc.teamcode.ftc2753.teleop;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ftc2753.subsystems.DriveTrain;

import java.util.Locale;

@Config
@TeleOp(name = "Teleop", group = "TeleOp")
public class Teleop2 extends LinearOpMode {

    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor intake;

    private Servo foundationLeft;
    private Servo foundationRight;
    private Servo intakeLift;
    private Servo armClaw;
    private Servo armLeft, armRight;

    private float turnSpeed = 1;

    private float intakeSpeed;
    private float speed = 1;

    boolean wasdpad_down = false;
    boolean wasdpad_up = false;
    boolean wasdpad_left = false;
    boolean wasdpad_right = false;
    boolean wasXDOWN = false;



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

    public int ybuttonNum = 0;
    public boolean wasyDown = false;


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
        intakeLift.setPosition(1);

        waitForStart();

        setBrake(motorFrontLeft);
        setBrake(motorFrontRight);
        setBrake(motorBackLeft);
        setBrake(motorBackRight);

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.y) {
                imu = hardwareMap.get(BNO055IMU.class, "imu");
                imu.initialize(parameters);
            }

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

            drive.BackLeft -= gamepad1.left_trigger;
            drive.FrontLeft -= gamepad1.left_trigger;
            drive.BackRight -= gamepad1.right_trigger;
            drive.FrontRight -= gamepad1.right_trigger;

            if (gamepad1.dpad_down) {
                drive.BackLeft += 2 * gamepad1.left_trigger;
                drive.FrontLeft += 2 * gamepad1.left_trigger;
                drive.BackRight += 2 * gamepad1.right_trigger;
                drive.FrontRight += 2 * gamepad1.right_trigger;
            }

            if(gamepad1.x)
                grabFoundation();
            if(gamepad1.b)
                releaseFoundation();
            if (gamepad1.a) {
                foundationRight.setPosition(0.0f);
                foundationLeft.setPosition(1.0f);
            }
            if (gamepad2.right_bumper) {
                intakeLift.setPosition(0.6);//Raise intake
            }
            if (gamepad2.left_bumper) {
                intakeLift.setPosition(0);//Lower intake
            }

            if (gamepad2.y)
                setArmPosition((float) (armRight.getPosition() + 0.005));
            if (gamepad2.a)
                setArmPosition((float) (armRight.getPosition() - 0.005));
            if (gamepad2.x) {
                armClaw.setPosition(0.8);
            }
            if (gamepad2.b) {
                armClaw.setPosition(0.25);
            }
            update();
            //Telemetry
            telemetry.addData("Heading", formatAngle(angles.angleUnit, angles.firstAngle));
            telemetry.addData("back left ",drive.BackLeft);
            telemetry.addData("front left ",drive.FrontLeft);
            telemetry.addData("front right ",drive.FrontRight);
            telemetry.addData("back right ",drive.BackRight);
            telemetry.update();
        }
        requestOpModeStop();
    }

    private void grabFoundation() {
        foundationRight.setPosition(1.0f);
        foundationLeft.setPosition(0.0f);
    }

    private void releaseFoundation(){
        foundationRight.setPosition(0.5f);
        foundationLeft.setPosition(0.5f);
    }

    private void setArmPosition(float position){
        armRight.setPosition(position);
        armLeft.setPosition(1-position);
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
        foundationLeft = hardwareMap.get(ServoImplEx.class, "foundationLeft");
        foundationRight = hardwareMap.get(ServoImplEx.class, "foundationRight");
        intakeLift = hardwareMap.get(ServoImplEx.class, "liftIntake");
        armClaw = hardwareMap.get(ServoImplEx.class, "ArmClaw");
        armLeft = hardwareMap.get(ServoImplEx.class, "ArmLeft");
        armRight = hardwareMap.get(ServoImplEx.class, "ArmRight");
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

        drive.move(relativeAngle, Math.sqrt(gamepad1.left_stick_x * gamepad1.left_stick_x +  gamepad1.left_stick_y * gamepad1.left_stick_y) * speed,
                gamepad1.right_stick_x * turnSpeed);
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
    public BNO055IMU returnIMU(BNO055IMU.Parameters parameters) {
        BNO055IMU imu = null;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        return imu;
    }

}



