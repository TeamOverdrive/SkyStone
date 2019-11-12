package org.firstinspires.ftc.teamcode.ftc2753.subsystems;

import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.ftc2753.subsystems.DriveTrain;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;

public class IMUTest extends LinearOpMode {


    @Autonomous(name = "Blue Foundation", group = "auto")


    DriveTrain drive = new DriveTrain();

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime waittime = new ElapsedTime();

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
    double globalAngle, power = .30, correction;

    private Servo sensorRotator;
    private Servo foundationLeft;
    private Servo foundationRight;

    private float intakeSpeed;

    /*
    note that with the config annotation above public class Teleop2,
    the below public static non-final variables can be edited on the fly with FTC Dashboard
    */
    //sideUp and sideDown are used for both the stone side grabber and the sensor array actuator
    public static double sideUp = 180 / 270;
    public static double sideDown = 0 / 270;
    public static double grabberDiagnostic = 0.5; //grabber is in continuous mode rn

    //values for the foundation grabber servos; note foundationGrab() and foundationRelease() methods
    public static double foundationUp = 1;
    public static double foundationDown = 0;
    public static double foundationDiagnostic = 0.5;

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
        BNO055IMU imu = initIMU();
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        foundationLeft.setPosition(0.5f);
        foundationRight.setPosition(0.5f);

        // Wait for the start button to be pressed.
        waitForStart();
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        turnTo(80, imu);
        // Loop and update the dashboard
        /*while (opModeIsActive()) { COMMENTED OUT COMMENT BACK IN
            telemetry.update();
        }
        moveInch(-20,0.2f,4);
        drive.move(0);
        update();

        strafeInch(-10,0.5f,7);

        moveInch(-14,0.2f,4);

        drive.move(0);
        update();
        foundationLeft.setPosition(0.0f);
        foundationRight.setPosition(1.0f);

        sleep(1000);

        moveInch(38,0.6f,10);

        drive.move(0);
        update();

        sleep(500);

        foundationLeft.setPosition(0.5f);
        foundationRight.setPosition(0.5f);

        sleep(15000);

        strafeInch(58,1,3);

         */
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

    public void stopMove() {

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }

    public void moveInch(int inches, float speed, float timeout) {

        int motorFrontRightTP = motorFrontRight.getCurrentPosition() + (int) (inches * drive.COUNTS_PER_INCH);
        int motorBackRightTP = motorBackRight.getCurrentPosition() + (int) (inches * drive.COUNTS_PER_INCH);
        int motorFrontLeftTP = motorFrontLeft.getCurrentPosition() + (int) (inches * drive.COUNTS_PER_INCH);
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
                (this.runtime.seconds() < timeout) &&
                (motorFrontRight.isBusy() && motorBackRight.isBusy() && motorFrontLeft.isBusy() && motorBackLeft.isBusy())) {

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

    public void turnTo(double angle, BNO055IMU imu) {

        Orientation angles;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, XYZ, AngleUnit.DEGREES);

        while (!(angle <= angles.firstAngle + 1) && !(angle >= angles.firstAngle - 1)) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);
            double relativeTarget = Math.toRadians(angle) - Math.toRadians(angles.firstAngle);
            if (Math.abs(relativeTarget) > Math.PI) {
                if (relativeTarget > 0)
                    relativeTarget = -(Math.PI * 2 - Math.abs(relativeTarget));
                else if (relativeTarget < 0)
                    relativeTarget = Math.PI * 2 - Math.abs(relativeTarget);
            }

            motorFrontRight.setPower((relativeTarget / Math.PI) * Math.abs(relativeTarget / Math.PI));
            motorBackRight.setPower((relativeTarget / Math.PI) * Math.abs(relativeTarget / Math.PI));
            motorFrontLeft.setPower(-((relativeTarget / Math.PI) * Math.abs(relativeTarget / Math.PI)));
            motorBackLeft.setPower(-((relativeTarget / Math.PI) * Math.abs(relativeTarget / Math.PI)));


        }

    }

    public void moveInch(int inchesLeft, int inchesRight, float speed, float timeout) {

        int motorFrontRightTP = motorFrontRight.getCurrentPosition() + (int) (inchesRight * drive.COUNTS_PER_INCH);
        int motorBackRightTP = motorBackRight.getCurrentPosition() + (int) (inchesRight * drive.COUNTS_PER_INCH);
        int motorFrontLeftTP = motorFrontLeft.getCurrentPosition() + (int) (inchesLeft * drive.COUNTS_PER_INCH);
        int motorBackLeftTP = motorBackLeft.getCurrentPosition() + (int) (inchesLeft * drive.COUNTS_PER_INCH);

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
                (this.runtime.seconds() < timeout) &&
                (motorFrontRight.isBusy() && motorBackRight.isBusy() && motorFrontLeft.isBusy() && motorBackLeft.isBusy())) {

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

    public BNO055IMU initIMU() {
        BNO055IMU imu;
        Orientation angles;
        Acceleration gravity;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        return imu;
    }

    //Everything after this is copied
    private void resetAngle(BNO055IMU imu) {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle(BNO055IMU imu) {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */

    private void rotate(int degrees, double power, BNO055IMU imu) {
        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle(imu);

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        // set power to rotate.
        motorBackLeft.setPower(leftPower);
        motorFrontLeft.setPower(leftPower);
        motorBackRight.setPower(rightPower);
        motorFrontRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle(imu) == 0) {
            }

            while (opModeIsActive() && getAngle(imu) > degrees) {
            }
        } else    // left turn.
            while (opModeIsActive() && getAngle(imu) < degrees) {
            }

        // turn the motors off.
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);

        // wait for rotation to stop.
        sleep(400);

        // reset angle tracking on new heading.
        resetAngle(imu);
    }

    public void initServos() {
        sideGrabber = hardwareMap.get(ServoImplEx.class, "sideGrabber");
        sensorRotator = hardwareMap.get(ServoImplEx.class, "sensor");
        foundationLeft = hardwareMap.get(ServoImplEx.class, "foundationLeft");
        foundationRight = hardwareMap.get(ServoImplEx.class, "foundationRight");
    }

    public void strafeInch(int inches, float speed, float timeout) {

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
                (this.runtime.seconds() < timeout) &&
                (motorFrontRight.isBusy() && motorBackRight.isBusy() && motorFrontLeft.isBusy() && motorBackLeft.isBusy())) {

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


}
}
