package org.firstinspires.ftc.teamcode.ftc2753.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ftc2753.subsystems.DriveTrain;

@TeleOp(name = "Create Auto", group = "autos")
public class AutoTesting extends LinearOpMode {

    String output[] = new String[100];
    int currentPosition = 0;
    DcMotor motorBackLeft,motorBackRight,motorFrontLeft,motorFrontRight,intake;
    int distanceTraveled = 0;
    DriveTrain drive = new DriveTrain();
    BNO055IMU imu;
    ElapsedTime runtime = new ElapsedTime();

    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
    static final double HEADING_THRESHOLD = 1;

    @Override
    public void runOpMode() {
        Orientation angles;


        initMotors();
        initIMU();
        while (opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if (gamepad1.y) {
                distanceTraveled = 0;
                while (gamepad1.y) {
                    moveInch(1, 0.1, angles.firstAngle, angles);
                    distanceTraveled++;
                }
                output[currentPosition] = "Move Foward: " + String.valueOf(distanceTraveled);
                distanceTraveled = 0;
                currentPosition++;

            }
            if (gamepad1.b) {
                distanceTraveled = 0;
                while (gamepad1.b) {
                    strafeInch(1, 0.1f, 10);
                    distanceTraveled++;
                }
                output[currentPosition] = "Strafe: " + String.valueOf(distanceTraveled);
                distanceTraveled = 0;
                currentPosition++;

            }
            if (gamepad1.a) {
                distanceTraveled = 0;
                while (gamepad1.a) {
                    strafeInch(-1, 0.1f, 10);
                    distanceTraveled++;
                }
                output[currentPosition] = "Strafe: " + String.valueOf(-distanceTraveled);
                distanceTraveled = 0;
                currentPosition++;

            }
            if (gamepad1.x) {
                distanceTraveled = 0;
                while (gamepad1.x) {
                    moveInch(-1, 0.1f, angles.firstAngle, angles);
                    distanceTraveled++;
                }
                output[currentPosition] = "Move Back: " + String.valueOf(-distanceTraveled);
                distanceTraveled = 0;
                currentPosition++;

            }
            motorFrontRight.setPower(-gamepad1.left_stick_x);
            motorBackRight.setPower(-gamepad1.left_stick_x);
            motorFrontLeft.setPower(gamepad1.left_stick_x);
            motorBackLeft.setPower(gamepad1.left_stick_x);
            if (gamepad1.left_bumper) {
                output[currentPosition] = "Turn to: " + String.valueOf(angles.firstAngle);
                currentPosition++;
            }
            for (int i = 0; i < 100; i++) {
                if (output[i] != null)
                    telemetry.addLine(output[i]);
            }
            telemetry.update();


        }
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
    public void moveInch ( int inches,
                           double speed,
                           double angle, Orientation angles) {

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

                // adjust relative speed based on heading error.
                error = drive.getError(angle,angles);
                steer = drive.getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (inches < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

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
