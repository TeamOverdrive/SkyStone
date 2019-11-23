package org.firstinspires.ftc.teamcode.ftc2753.auto;

import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ftc2753.subsystems.DriveTrain;

@Autonomous(name="Red Foundation", group="auto")

public class RedFoundation extends LinearOpMode {

    DriveTrain drive = new DriveTrain();

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime waittime = new ElapsedTime();

    BNO055IMU imu;

    private DistanceSensor distRight;
    private DistanceSensor distLeft;

    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;

    private Servo sideGrabber;

    boolean wasYDown = false;

    boolean targetFound = false;
    Orientation lastAngles = new Orientation();
    double globalAngle;

    private Servo sensorRotator;
    private Servo foundationLeft;
    private Servo foundationRight;
    private Servo intakeLift;

    private float intakeSpeed;

    Orientation angles;

    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro


    NormalizedColorSensor colorSensor;
    /**
     * The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need something analogous when you
     * use a color sensor on your robot
     */
    View relativeLayout;

    public void runOpMode() throws InterruptedException {

        initMotors();
        initServos();
        initIMU();

        foundationLeft.setPosition(0.1f);
        foundationRight.setPosition(0.0f);

        // Wait for the start button to be pressed.
        waitForStart();

        moveInch(-20, 0.2, 0);
        drive.move(0);
        update();

        strafeInch(10, 0.5f, 7);

        moveInch(-14, 0.2f, 0);

        drive.move(0);
        update();
        foundationLeft.setPosition(0.5f);
        foundationRight.setPosition(0.5f);

        sleep(1000);

        moveInch(30, 0.6f, 0);

        strafeInch(8, 0.6f, 3);

        gyroTurn(0.4f, 90);

        moveInch(-6, 0.4f, 90);

        strafeInch(12, 0.6f, 2);

        drive.move(0);
        update();

        sleep(500);

        foundationLeft.setPosition(1f);
        foundationRight.setPosition(0f);

        strafeInch(12, 0.6f, 3);

        sleep(15000);

        moveInch(34, 1, 90);
        update();

    }

    public void initMotors() {

        motorBackLeft = hardwareMap.get(DcMotor.class, "left_back");
        motorBackRight = hardwareMap.get(DcMotor.class, "right_back");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "left_front");
        motorFrontRight = hardwareMap.get(DcMotor.class, "right_front");

        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);

    }

    public void update() {
        motorFrontLeft.setPower(drive.FrontLeft);
        motorFrontRight.setPower(drive.FrontRight);
        motorBackLeft.setPower(drive.BackLeft);
        motorBackRight.setPower(drive.BackRight);

    }

    public void setBrake() {

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void removeBrake() {

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
    public void moveInch ( int inches,
                            double speed,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int motorFrontRightTP = motorFrontRight.getCurrentPosition() + (int) (inches * drive.COUNTS_PER_INCH);
            int motorBackRightTP = motorBackRight.getCurrentPosition() + (int) (inches * drive.COUNTS_PER_INCH);
            int motorFrontLeftTP = motorFrontLeft.getCurrentPosition() + (int) (inches * drive.COUNTS_PER_INCH);
            int motorBackLeftTP = motorBackLeft.getCurrentPosition() + (int) (inches * drive.COUNTS_PER_INCH);


            // Set Target and Turn On RUN_TO_POSITION
            motorFrontRight.setTargetPosition(motorFrontRightTP);
            motorBackRight.setTargetPosition(motorBackRightTP);
            motorFrontLeft.setTargetPosition(motorFrontLeftTP);
            motorBackLeft.setTargetPosition(motorBackLeftTP);

            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);

            runtime.reset();

            motorFrontRight.setPower(Math.abs(speed));
            motorBackRight.setPower(Math.abs(speed));
            motorFrontLeft.setPower(Math.abs(speed));
            motorBackLeft.setPower(Math.abs(speed));

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (motorFrontRight.isBusy() && motorFrontLeft.isBusy())) {

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                // adjust relative speed based on heading error.
                error = drive.getError(angle,angles);
                steer = drive.getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (inches < 0)
                    steer *= -1.0;

                leftSpeed = speed - (steer * 0.1);
                rightSpeed = speed + (steer * 0.1);

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                motorFrontRight.setPower(Math.abs(rightSpeed));
                motorBackRight.setPower(Math.abs(rightSpeed));
                motorFrontLeft.setPower(Math.abs(leftSpeed));
                motorBackLeft.setPower(Math.abs(leftSpeed));

                // Display drive status for the driver.
            }

            // Stop all motion;
            motorFrontRight.setPower(0);
            motorBackRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);

            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
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

    //Everything after this is copied
    private void resetAngle(BNO055IMU imu) {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public void strafeInch(int inches, float speed, float angle) {

        Orientation angles;

        double  max;
        double  error;
        double  steer;

        int motorFrontRightTP = motorFrontRight.getCurrentPosition() + (int) (inches * drive.COUNTS_PER_INCH);
        int motorBackRightTP = motorBackRight.getCurrentPosition() + (int) (-inches * drive.COUNTS_PER_INCH);
        int motorFrontLeftTP = motorFrontLeft.getCurrentPosition() + (int) (-inches * drive.COUNTS_PER_INCH);
        int motorBackLeftTP = motorBackLeft.getCurrentPosition() + (int) (inches * drive.COUNTS_PER_INCH);

        motorFrontRight.setTargetPosition(motorFrontRightTP);
        motorBackRight.setTargetPosition(motorBackRightTP);
        motorFrontLeft.setTargetPosition(motorFrontLeftTP);
        motorBackLeft.setTargetPosition(motorBackLeftTP);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.runtime.reset();

        motorFrontRight.setPower(Math.abs(speed));
        motorBackRight.setPower(Math.abs(speed));
        motorFrontLeft.setPower(Math.abs(speed));
        motorBackLeft.setPower(Math.abs(speed));


        while (opModeIsActive() &&
                (motorFrontRight.isBusy() && motorBackRight.isBusy() && motorFrontLeft.isBusy() && motorBackLeft.isBusy())) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            // adjust relative speed based on heading error.
            error = drive.getError(angle,angles);
            steer = drive.getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (inches < 0)
                steer *= -1.0;

            motorFrontRight.setPower(Math.abs(speed + (steer * 0.1)));
            motorBackRight.setPower(Math.abs(speed + (steer * 0.1)));
            motorFrontLeft.setPower(Math.abs(speed - (steer * 0.1)));
            motorBackLeft.setPower(Math.abs(speed - (steer * 0.1)));

        }

        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF, angles)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
    }
    public boolean onHeading(double speed, double angle, double PCoeff, Orientation angles) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = drive.getError(angle,angles);

        if (Math.abs(error) <= this.HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = drive.getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        motorBackLeft.setPower(leftSpeed);
        motorFrontLeft.setPower(leftSpeed);
        motorBackRight.setPower(rightSpeed);
        motorFrontRight.setPower(rightSpeed);

        return onTarget;
    }
    public void initServos(){
        sideGrabber = hardwareMap.get(ServoImplEx.class, "sideGrabber");
        sensorRotator = hardwareMap.get(ServoImplEx.class, "sensor");
        foundationLeft = hardwareMap.get(ServoImplEx.class, "foundationLeft");
        foundationRight = hardwareMap.get(ServoImplEx.class, "foundationRight");
        intakeLift = hardwareMap.get(ServoImplEx.class, "liftIntake");
    }
}