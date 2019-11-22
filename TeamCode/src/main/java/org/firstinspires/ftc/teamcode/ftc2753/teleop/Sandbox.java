package org.firstinspires.ftc.teamcode.ftc2753.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ftc2753.subsystems.DriveTrain;

import static org.firstinspires.ftc.teamcode.ftc2753.teleop.AutoTesting.P_DRIVE_COEFF;

@TeleOp(name="Test Me", group = "tests")
public class Sandbox extends LinearOpMode {

    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;

    private Servo sideGrabber;
    private Servo foundationLeft;
    private Servo foundationRight;

    DriveTrain drive = new DriveTrain();

    BNO055IMU imu;

    Orientation angles;

    public void runOpMode() {

        initMotors();

        waitForStart();

        sideGrabber = hardwareMap.get(ServoImplEx.class, "sideGrabber");
        foundationLeft = hardwareMap.get(ServoImplEx.class, "foundationLeft");
        foundationRight = hardwareMap.get(ServoImplEx.class, "foundationRight");

        ElapsedTime runtime = new ElapsedTime();
        initIMU();
        strafeInch(64,0.6f,0);

    }
    public void initMotors() {

        motorBackLeft = hardwareMap.get(DcMotor.class, "left_back");
        motorBackRight = hardwareMap.get(DcMotor.class, "right_back");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "left_front");
        motorFrontRight = hardwareMap.get(DcMotor.class, "right_front"); // why is this here

        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
}
