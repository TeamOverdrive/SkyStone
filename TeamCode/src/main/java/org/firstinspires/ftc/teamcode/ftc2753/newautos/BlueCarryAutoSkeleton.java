package org.firstinspires.ftc.teamcode.ftc2753.newautos;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ftc2753.subsystems.DriveTrain;

@Autonomous(name="Red Skystone", group="auto")
public class BlueCarryAutoSkeleton extends LinearOpMode {

    DriveTrain drive = new DriveTrain();

    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU imu;

    private DistanceSensor distRight;
    private DistanceSensor distLeft;

    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    Boolean targetFound = false;

    Orientation lastAngles = new Orientation();
    double globalAngle;

    private Servo sensorRotator;
    private Servo foundationLeft;
    private Servo foundationRight;
    private Servo intakeLift;
    private Servo leftArm;
    private Servo rightArm;
    private Servo grabber;


    Orientation angles;

    float[] hsvValues = new float[3];

     int skystonePosition = 1;
     boolean Switch = true;
     public double Potato;


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

        setBrake();

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        distRight = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)distRight;

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        intakeLift.setPosition(1);

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        waitForStart();

        grabber.setPosition(0);

        foundationLeft.setPosition(1f);
        foundationRight.setPosition(0f);

        setArmPosition(0.5f);
        moveInch(-25,0.3, 0); // initial move forwards
        strafeInch(-25,0.4f, 0);
        gyroTurn(0.2,0);

        if (Switch == true) {
            while (distRight.getDistance(DistanceUnit.MM) > 120) {
                drive.move(-0.05f);
                update();

                //telemetry.addLine(Potato);
            }
        }
        else {
            drive.move(0);
            update();
        }

        while (!targetFound) {

            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            Color.colorToHSV(colors.toColor(), hsvValues);
            int color = colors.toColor();
            sleep(350);

            telemetry.addData("Color: ", Color.red(color));
            telemetry.update();
            if (Color.red((color)) > 0.75 && (skystonePosition != 5)) {  // if yellow....
                /*if (distRight.getDistance(DistanceUnit.MM) < 90) {
                    moveInch(1,0.05f,0);
                }

                 */
               strafeInch(8,0.20f,0);
               //gyroTurn(0.4,0);
               skystonePosition ++;
               telemetry.addLine("Skystone Not Detected");
            } else {  //if not yellow...
                telemetry.addLine("SKYSTONE Detected");

                drive.move(0);
                update();
                targetFound = true;
                Switch = false;
                moveInch(2,0.25,0);
                strafeInch(3,0.3f,0);


                //moveInch(3,0.25,0); //moves back after detecting skystone

            }

            telemetry.addData("R: ", Color.red(color));
            telemetry.addData("Position: ", skystonePosition);
            telemetry.update();
        }


        pickUp();

        // Position to move over bridge
        moveInch(4,0.4f,0);
        gyroTurn(0.1,-90);
        strafeInch(-2,0.4f,-90);
        gyroTurn(0.1,-90);

        // Move over bridge
        moveInch(-(108 - (skystonePosition * 8)), 0.8,-90);

        // Turn and grab foundation
        gyroTurn(0.25f,0);
        strafeInch(7,0.7f,0);
        moveInch(-14,0.2,0);
        foundationLeft.setPosition(0.5);
        foundationRight.setPosition(0.5);
        // setArmPosition(1);
        sleep(500);

        // Drop block
        grabber.setPosition(0);
        sleep(1000);

        // Move back while pulling the foundation
        moveInch(44,1,0);

        // Reset Arm to inside robot
        setArmPosition(0);

        // Move to maximise leverage and then turn with foundation
        strafeInch(20,0.5f,0);
        gyroTurn(1,-90);

        // Push it towards the wall and release
        moveInch(-20,1,-90);
        foundationLeft.setPosition(1f);
        foundationRight.setPosition(0f);
        sleep(500);

        // Move back under bridge
        strafeInch(-3,1,-90);
        gyroTurn(0.3,-90);
        moveInch(40,0.8,-90);


    }

    public void pickUp() {
        setArmPosition(1);
        sleep(1000);
        grabber.setPosition(1);
        sleep(1000);
        setArmPosition(0.75f);
        sleep(500);


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

        Orientation angles;

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
        sensorRotator = hardwareMap.get(ServoImplEx.class, "sensor");
        foundationLeft = hardwareMap.get(ServoImplEx.class, "foundationLeft");
        foundationRight = hardwareMap.get(ServoImplEx.class, "foundationRight");
        intakeLift = hardwareMap.get(ServoImplEx.class, "liftIntake");
        leftArm = hardwareMap.get(ServoImplEx.class, "ArmLeft");
        rightArm = hardwareMap.get(ServoImplEx.class, "ArmRight");
        grabber = hardwareMap.get(ServoImplEx.class, "ArmClaw");
    }
    private void setArmPosition(float position){
        leftArm.setPosition(position);
        rightArm.setPosition(1-position);
    }
}