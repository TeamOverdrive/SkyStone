package org.firstinspires.ftc.teamcode.ftc2753.auto;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ftc2753.subsystems.DriveTrain;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;

@Disabled
@Config
@Autonomous(name="Blue Skystone", group="auto")

public class BlueSkystone extends LinearOpMode {

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
    private Servo sensorRotator;
    private Servo foundationLeft;
    private Servo foundationRight;
    private Servo intakeLift;

    public static double GRABBERUP = 0.66;
    public static double GRABBERDOWN = 0.8;

    boolean wasYDown = false;

    boolean targetFound = false;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;

    NormalizedColorSensor colorSensor;
    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need something analogous when you
     * use a color sensor on your robot */
    View relativeLayout;

    public void runOpMode() throws InterruptedException {

        initMotors();
        initServos();

        sideGrabber.setPosition(0.5f);
        intakeLift.setPosition(1);



        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        distRight = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)distRight;

        float[] hsvValues = new float[3];
        final float values[] = hsvValues;

        // bPrevState and bCurrState keep track of the previous and current state of the button
        boolean bPrevState = false;
        boolean bCurrState = false;

        // Get a reference to our sensor object.
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        Orientation angles;

        // Wait for the start button to be pressed.
        waitForStart();

        strafeInch(24,0.95f,7);

        moveInch(28,1,2);

        strafeInch(9,0.95f,7);
        while (distRight.getDistance(DistanceUnit.MM) > 97) {
            drive.move("LEFT",0.2f);
            update();
        }

        drive.move(0);
        update();

        sleep(500);

        moveInch(-18,0.9f,3);

        while (!targetFound) {

            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            /** Use telemetry to display feedback on the driver station. We show the conversion
             * of the colors to hue, saturation and value, and display the the normalized values
             * as returned from the sensor.
             * @see <a href="http://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html">HSV</a>*/

            Color.colorToHSV(colors.toColor(), hsvValues);
            telemetry.addLine()
                    .addData("H", "%.3f", hsvValues[0])
                    .addData("S", "%.3f", hsvValues[1])
                    .addData("V", "%.3f", hsvValues[2]);
            telemetry.addLine()
                    .addData("a", "%.3f", colors.alpha)
                    .addData("r", "%.3f", colors.red)
                    .addData("g", "%.3f", colors.green)
                    .addData("b", "%.3f", colors.blue);

            /** We also display a conversion of the colors to an equivalent Android color integer.
             * @see Color */
            int color = colors.toColor();
            telemetry.addLine("raw Android color: ")
                    .addData("a", "%02x", Color.alpha(color))
                    .addData("r", "%02x", Color.red(color))
                    .addData("g", "%02x", Color.green(color))
                    .addData("b", "%02x", Color.blue(color));
            telemetry.addData("RightDist: ", distRight.getDistance(DistanceUnit.MM));
            if (Color.red((color)) > 14) {
                telemetry.addLine("Nope");
            } else {
                telemetry.addLine("Skystone");
            }
            if (Color.red((color)) > 0) {
                drive.move(-0.2f);
                update();
            } else {
                drive.move(0);
                update();
                targetFound = true;

            }


            // Balance the colors. The values returned by getColors() are normalized relative to the
            // maximum possible values that the sensor can measure. For example, a sensor might in a
            // particular configuration be able to internally measure color intensity in a range of
            // [0, 10240]. In such a case, the values returned by getColors() will be divided by 10240
            // so as to return a value it the range [0,1]. However, and this is the point, even so, the
            // values we see here may not get close to 1.0 in, e.g., low light conditions where the
            // sensor measurements don't approach their maximum limit. In such situations, the *relative*
            // intensities of the colors are likely what is most interesting. Here, for example, we boost
            // the signal on the colors while maintaining their relative balance so as to give more
            // vibrant visual feedback on the robot controller visual display.
            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red   /= max;
            colors.green /= max;
            colors.blue  /= max;
            color = colors.toColor();

            telemetry.addLine("normalized color:  ")
                    .addData("a", "%02x", Color.alpha(color))
                    .addData("r", "%02x", Color.red(color))
                    .addData("g", "%02x", Color.green(color))
                    .addData("b", "%02x", Color.blue(color));
            telemetry.update();
        }
        moveInch(8,0.2f,5);

        // sleep(1000);

        strafeInch(3,0.3f,1);

        sideGrabber.setPosition(0);

        sleep(1000);

        strafeInch(-17,0.3f,100);

        moveInch(-74,0.5f,100);

        sideGrabber.setPosition(0.5f);

        sleep(1000);

        moveInch(105,0.4f,100);

        moveInch(-58,0.6f,100);

        strafeInch(18,0.3f,3);



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

        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int motorFrontRightTP = motorFrontRight.getCurrentPosition() + (int)(inches * drive.COUNTS_PER_INCH);
        int motorBackRightTP = motorBackRight.getCurrentPosition() + (int)(inches * drive.COUNTS_PER_INCH);
        int motorFrontLeftTP = motorFrontLeft.getCurrentPosition() + (int)(inches * drive.COUNTS_PER_INCH);
        int motorBackLeftTP = motorBackLeft.getCurrentPosition() + (int)(inches * drive.COUNTS_PER_INCH);

        motorFrontRight.setTargetPosition(motorFrontRightTP);
        motorBackRight.setTargetPosition(motorBackRightTP);
        motorFrontLeft.setTargetPosition(motorFrontLeftTP);
        motorBackLeft.setTargetPosition(motorBackLeftTP);

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
    public void turnTo(double angle,BNO055IMU imu) {

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
            //what is the issue here, it does not want to work on the math.pi nigerian
            motorFrontRight.setPower((relativeTarget/Math.PI) * Math.abs(relativeTarget/Math.PI));
            motorBackRight.setPower((relativeTarget/Math.PI) * Math.abs(relativeTarget/Math.PI));
            motorFrontLeft.setPower(-((relativeTarget/Math.PI) * Math.abs(relativeTarget/Math.PI)));
            motorBackLeft.setPower(-((relativeTarget/Math.PI) * Math.abs(relativeTarget/Math.PI)));


        }

    }

    public void moveInch(int inchesLeft,int inchesRight, float speed, float timeout) {

        int motorFrontRightTP = motorFrontRight.getCurrentPosition() + (int)(inchesRight * drive.COUNTS_PER_INCH);
        int motorBackRightTP = motorBackRight.getCurrentPosition() + (int)(inchesRight * drive.COUNTS_PER_INCH);
        int motorFrontLeftTP = motorFrontLeft.getCurrentPosition() + (int)(inchesLeft * drive.COUNTS_PER_INCH);
        int motorBackLeftTP = motorBackLeft.getCurrentPosition() + (int)(inchesLeft * drive.COUNTS_PER_INCH);

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

        BNO055IMU.Parameters IMUparameters = new BNO055IMU.Parameters();
        IMUparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMUparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUparameters.calibrationDataFile = "BNO055IMUCalibration.json";
        IMUparameters.loggingEnabled = true;
        IMUparameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUparameters);

        return imu;
    }

    //Everything after this is copied
    private void resetAngle(BNO055IMU imu)
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle(BNO055IMU imu)
    {
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
     * @return Power adjustment, + is adjust left - is adjust right.
     */

    private void rotate(int degrees, double power,BNO055IMU imu)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle(imu);

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        motorBackLeft.setPower(leftPower);
        motorFrontLeft.setPower(leftPower);
        motorBackRight.setPower(rightPower);
        motorFrontRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle(imu) == 0) {}

            while (opModeIsActive() && getAngle(imu) > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle(imu) < degrees) {}

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
    public void initServos(){
        sideGrabber = hardwareMap.get(ServoImplEx.class, "sideGrabber");
        sensorRotator = hardwareMap.get(ServoImplEx.class, "sensor");
        foundationLeft = hardwareMap.get(ServoImplEx.class, "foundationLeft");
        foundationRight = hardwareMap.get(ServoImplEx.class, "foundationRight");
        intakeLift = hardwareMap.get(ServoImplEx.class, "liftIntake");
    }
    public void strafeInch(int inches, float speed, float timeout) {

        int motorFrontRightTP = motorFrontRight.getCurrentPosition() + (int)(inches * drive.COUNTS_PER_INCH);
        int motorBackRightTP = motorBackRight.getCurrentPosition() + (int)(-inches * drive.COUNTS_PER_INCH);
        int motorFrontLeftTP = motorFrontLeft.getCurrentPosition() + (int)(-inches * drive.COUNTS_PER_INCH);
        int motorBackLeftTP = motorBackLeft.getCurrentPosition() + (int)(inches * drive.COUNTS_PER_INCH);

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
