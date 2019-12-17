package org.firstinspires.ftc.teamcode.Skystone2753;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.Arrays;
import java.util.List;

public class driveTrain extends Robot {

    public DcMotor frontLeft,frontRight,backLeft,backRight;
    private DcMotor[] motors = {frontLeft,frontRight,backLeft,backRight};
    public double frontLeftPower, frontRightPower,backLeftPower,backRightPower;
    private double[] powers = {frontLeftPower, frontRightPower,backLeftPower,backRightPower};

    coefficients driveK = new coefficients();
    DistanceUnit INCHES = new DistanceUnit(43.465342326685739);
    DistanceUnit MM = new DistanceUnit( 1.711233949);
    DistanceUnit CM = new DistanceUnit(17.11233949);
    DistanceUnit FEET = new DistanceUnit(521.5841084);
    double COUNTS;

    public driveTrain (LinearOpMode inLinearOpMode, BNO055IMU imu) {

        COUNTS = INCHES.COUNTS_PER_UNIT;
        super.linearOpMode = inLinearOpMode;
        super.imu = imu;

    }

    public void initDrive() {
        super.linearOpMode.telemetry.addLine("InitDrive");
        super.linearOpMode.telemetry.update();
        backLeft = super.linearOpMode.hardwareMap.get(DcMotor.class, "left_back");
        backRight = super.linearOpMode.hardwareMap.get(DcMotor.class, "right_back");
        frontLeft = super.linearOpMode.hardwareMap.get(DcMotor.class, "left_front");
        frontRight = super.linearOpMode.hardwareMap.get(DcMotor.class, "right_front");

        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);


    }
    public void brake() {
        for (int i = 0; i < motors.length; i++) {
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    public void brake(DcMotor motor) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void removeBrake() {
        for (int i = 0; i < motors.length; i++) {
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
    public void removeBrake(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void kill() {
        frontLeftPower = 0;
        frontRightPower = 0;
        backLeftPower = 0;
        backRightPower = 0;
    }
    public void setPower(double speed) {
        frontLeftPower = speed;
        frontRightPower = speed;
        backLeftPower = speed;
        backRightPower = speed;
    }
    public void setPower(double leftSpeed, double rightSpeed) {
        frontLeftPower = leftSpeed;
        frontRightPower = rightSpeed;
        backLeftPower = leftSpeed;
        backRightPower = rightSpeed;
    }
    public void setPower(double frontLeftPower, double frontRightPower,double backLeftPower,double backRightPower) {
        this.frontLeftPower = frontLeftPower;
        this.frontRightPower = frontRightPower;
        this.backLeftPower = backLeftPower;
        this.backRightPower = backRightPower;
    }
    public void setPower(double angle, double speed, double turn) {
        if (speed > 1)
            speed = 1;
        this.frontLeftPower = (speed * Math.cos(angle) * (2 / Math.sqrt(2))) + turn;
        this.frontRightPower = (speed * Math.sin(angle) * (2 / Math.sqrt(2))) - turn;
        this.backLeftPower = (speed * Math.sin(angle) * (2 / Math.sqrt(2))) + turn;
        this.backRightPower = (speed * Math.cos(angle) * (2 / Math.sqrt(2))) - turn;

    }
    public void setPower(String direction,float speed) {
        if (direction.equalsIgnoreCase("RIGHT")) {
            this.frontLeftPower = Math.abs(speed);
            this.frontRightPower = -Math.abs(speed);
            this.backLeftPower = -Math.abs(speed);
            this.backRightPower = Math.abs(speed);
        }
        if (direction.equalsIgnoreCase("LEFT")) {
            this.frontLeftPower = -Math.abs(speed);
            this.frontRightPower = Math.abs(speed);
            this.backLeftPower = Math.abs(speed);
            this.backRightPower = -Math.abs(speed);
        }
    }
    public void drive() {
        /*for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(powers[i]);
        }

         */
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);

    }
    public void move(double speed) {
        setPower(speed);
        drive();
    }
    public void move(double leftSpeed, double rightSpeed) {
        setPower(leftSpeed, rightSpeed);
        drive();
    }
    public void move(double frontLeftPower, double frontRightPower,double backLeftPower,double backRightPower) {
        setPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        drive();
    }
    public void move(double angle, double speed, double turn) {
        setPower(angle,speed,turn);
        drive();

    }
    public void move(String direction,float speed) {
        setPower(direction, speed);
        drive();
    }
    public void addPower(double speed) {
        frontLeftPower += speed;
        frontRightPower += speed;
        backLeftPower += speed;
        backRightPower += speed;
    }
    public void addPower(double leftSpeed, double rightSpeed) {
        frontLeftPower += leftSpeed;
        frontRightPower += rightSpeed;
        backLeftPower += leftSpeed;
        backRightPower += rightSpeed;
    }
    public void addPower(double frontLeftPower, double frontRightPower,double backLeftPower,double backRightPower) {
        this.frontLeftPower += frontLeftPower;
        this.frontRightPower += frontRightPower;
        this.backLeftPower += backLeftPower;
        this.backRightPower += backRightPower;
    }
    public void addPower(double angle, double speed, double turn) {
        if (speed > 1)
            speed = 1;
        this.frontLeftPower += (speed * Math.cos(angle) * (2 / Math.sqrt(2))) + turn;
        this.frontRightPower += (speed * Math.sin(angle) * (2 / Math.sqrt(2))) - turn;
        this.backLeftPower += (speed * Math.sin(angle) * (2 / Math.sqrt(2))) + turn;
        this.backRightPower += (speed * Math.cos(angle) * (2 / Math.sqrt(2))) - turn;

    }
    public void dilatePower(double dilation) {
        frontLeftPower *= dilation;
        frontRightPower *= dilation;
        backLeftPower *= dilation;
        backRightPower *= dilation;
    }
    public void dilatePower(double leftDilation, double rightDilation) {
        frontLeftPower *= leftDilation;
        frontRightPower *= rightDilation;
        backLeftPower *= leftDilation;
        backRightPower *= rightDilation;
    }
    public void dilatePower(double frontLeftDilation, double frontRightDilation,double backLeftDilation,double backRightDilation) {
        this.frontLeftPower *= frontLeftDilation;
        this.frontRightPower *= frontRightDilation;
        this.backLeftPower *= backLeftDilation;
        this.backRightPower *= backRightDilation;
    }
    public double getPower(DcMotor motor) {
        return motor.getPower();
    }
    /* public double[] getPower(){
        double[] returnArray = new double[4];
        for (int i = 0; i < motors.length; i++) {
            returnArray[i] = motors[i].getPower();
        }
        return returnArray;
    } */
    public double getError(double targetAngle, Orientation angles) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    public double clip(double speed) {
        return Range.clip(Math.abs(speed), 0.0, 1.0);
    }
    public void moveDist( int dist, double speed, double angle) {
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        Orientation angles;
        ElapsedTime runtime = new ElapsedTime();

        // Ensure that the opmode is still active
        if (super.linearOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int motorFrontRightTP = frontRight.getCurrentPosition() + (int) (dist * COUNTS);
            int motorBackRightTP = backRight.getCurrentPosition() + (int) (dist * COUNTS);
            int motorFrontLeftTP = frontLeft.getCurrentPosition() + (int) (dist * COUNTS);
            int motorBackLeftTP = backLeft.getCurrentPosition() + (int) (dist * COUNTS);


            // Set Target and Turn On RUN_TO_POSITION
            frontRight.setTargetPosition(motorFrontRightTP);
            backRight.setTargetPosition(motorBackRightTP);
            frontLeft.setTargetPosition(motorFrontLeftTP);
            backLeft.setTargetPosition(motorBackLeftTP);

            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = clip(speed);

            runtime.reset();

            move(Math.abs(speed));

            // keep looping while we are still active, and BOTH motors are running.
            while (super.linearOpMode.opModeIsActive() &&
                    (frontRight.isBusy() && frontLeft.isBusy())) {

                angles = super.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                // adjust relative speed based on heading error.
                error = this.getError(angle, angles);
                steer = this.getSteer(error, driveK.P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (dist < 0)
                    steer *= -1.0;

                leftSpeed = speed - (steer * 0.1);
                rightSpeed = speed + (steer * 0.1);

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                move(leftSpeed, rightSpeed);

            }
            this.kill();
        }
    }
    public void moveDist( int dist, double speed) {
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        Orientation angles;
        ElapsedTime runtime = new ElapsedTime();

        // Ensure that the opmode is still active
        if (super.linearOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            int motorFrontRightTP = frontRight.getCurrentPosition() + (int) (dist * COUNTS);
            int motorBackRightTP = backRight.getCurrentPosition() + (int) (dist * COUNTS);
            int motorFrontLeftTP = frontLeft.getCurrentPosition() + (int) (dist * COUNTS);
            int motorBackLeftTP = backLeft.getCurrentPosition() + (int) (dist * COUNTS);


            // Set Target and Turn On RUN_TO_POSITION
            frontRight.setTargetPosition(motorFrontRightTP);
            backRight.setTargetPosition(motorBackRightTP);
            frontLeft.setTargetPosition(motorFrontLeftTP);
            backLeft.setTargetPosition(motorBackLeftTP);

            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = clip(speed);

            runtime.reset();

            move(Math.abs(speed));

            angles = super.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double angle = angles.firstAngle;

            // keep looping while we are still active, and BOTH motors are running.
            while (super.linearOpMode.opModeIsActive() &&
                    (frontRight.isBusy() && frontLeft.isBusy())) {
                angles = super.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                // adjust relative speed based on heading error.
                error = this.getError(angle, angles);
                steer = this.getSteer(error, driveK.P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (dist < 0)
                    steer *= -1.0;

                leftSpeed = speed - (steer * 0.1);
                rightSpeed = speed + (steer * 0.1);

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                move(leftSpeed, rightSpeed);

            }
            this.kill();
        }
    }
    public void setUnit(DistanceUnit unit) {
        COUNTS = unit.COUNTS_PER_UNIT;
    }
}
