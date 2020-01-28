package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class driveTrain extends Robot {

    public DcMotor frontLeft,frontRight,backLeft,backRight;
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
        initIMU();

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
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void brake(DcMotor motor) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void removeBrake() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void removeBrake(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void kill(DcMotor motor) {
        motor.setPower(0);
    }
    public void kill() {
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
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
    public void setPower(String direction,double speed) {
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
    public void strafe(String direction,double speed) {
        setPower(direction, speed);
        drive();
    }
    public void strafe(String direction,double speed,double distance) {
        distance = Math.abs(distance);
        double  error;
        double  steer;
        double angle = angles.firstAngle;

        if (angles == null) {
            linearOpMode.telemetry.addLine("uh oh");
            return;
        }

        ElapsedTime runtime = new ElapsedTime();

        if (direction.equals("RIGHT")) {
            int motorFrontRightTP = frontRight.getCurrentPosition() + (int) (distance * COUNTS);
            int motorBackRightTP = backRight.getCurrentPosition() + (int) (-distance * COUNTS);
            int motorFrontLeftTP = frontLeft.getCurrentPosition() + (int) (-distance * COUNTS);
            int motorBackLeftTP = backLeft.getCurrentPosition() + (int) (distance * COUNTS);

            frontRight.setTargetPosition(motorFrontRightTP);
            backRight.setTargetPosition(motorBackRightTP);
            frontLeft.setTargetPosition(motorFrontLeftTP);
            backLeft.setTargetPosition(motorBackLeftTP);

            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            setPower(Math.abs(speed));


            while (linearOpMode.opModeIsActive() &&
                    (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy())) {

                // angles = super.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                // adjust relative speed based on heading error.
                error = getError(angle, angles);
                steer = getSteer(error, driveK.P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                frontRight.setPower(Math.abs(speed + (steer * 0.1)));
                backRight.setPower(Math.abs(speed + (steer * 0.1)));
                frontLeft.setPower(Math.abs(speed - (steer * 0.1)));
                backLeft.setPower(Math.abs(speed - (steer * 0.1)));

            }

            kill();

            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else if (direction.equals("LEFT")) {
            int motorFrontRightTP = frontRight.getCurrentPosition() + (int) (-distance * COUNTS);
            int motorBackRightTP = backRight.getCurrentPosition() + (int) (distance * COUNTS);
            int motorFrontLeftTP = frontLeft.getCurrentPosition() + (int) (distance * COUNTS);
            int motorBackLeftTP = backLeft.getCurrentPosition() + (int) (-distance * COUNTS);

            frontRight.setTargetPosition(motorFrontRightTP);
            backRight.setTargetPosition(motorBackRightTP);
            frontLeft.setTargetPosition(motorFrontLeftTP);
            backLeft.setTargetPosition(motorBackLeftTP);

            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            setPower(Math.abs(speed));


            while (linearOpMode.opModeIsActive() &&
                    (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy())) {

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                // adjust relative speed based on heading error.
                error = getError(angle, angles);
                steer = getSteer(error, driveK.P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                frontRight.setPower(Math.abs(speed + (steer * 0.1)));
                backRight.setPower(Math.abs(speed + (steer * 0.1)));
                frontLeft.setPower(Math.abs(speed - (steer * 0.1)));
                backLeft.setPower(Math.abs(speed - (steer * 0.1)));

            }

            kill();

            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
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
    private double getError(double targetAngle, Orientation angles) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    private double clip(double speed) {
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

            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

            double currentAngle = angles.firstAngle;

            // keep looping while we are still active, and BOTH motors are running.
            while (super.linearOpMode.opModeIsActive() &&
                    (frontRight.isBusy() || frontLeft.isBusy())) {
                angles = super.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                // adjust relative speed based on heading error.
                error = this.getError(currentAngle, angles);
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

            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void setUnit(DistanceUnit unit) {
        COUNTS = unit.COUNTS_PER_UNIT;
    }
    private boolean onHeading(double speed, double angle, double PCoeff, Orientation angles) {
        double   error = 0;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // determine turn power based on +/- error
        error = getError(angle,angles);

        if (Math.abs(error) <= driveK.HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        move(leftSpeed,rightSpeed);

        linearOpMode.telemetry.addData("LEFT", leftSpeed);
        linearOpMode.telemetry.addData("RIGHT", rightSpeed);
        linearOpMode.telemetry.update();

        return onTarget;
    }
    private void turnTo(double speed, double angle) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        while (linearOpMode.opModeIsActive() && !onHeading(speed, angle, driveK.P_TURN_COEFF, angles)) {
            // Update telemetry & Allow time for other processes to run.
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }

    }
    public void turn(double speed, double angle) {
        turnTo(speed,angle);
        linearOpMode.sleep(100);
        turnTo(0.2,angle);
    }
    public void moveDist( int dist, double speed, double angle, boolean decel) {
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

            int motorFrontRightPos = frontRight.getCurrentPosition();
            int motorBackRightPos = backRight.getCurrentPosition();
            int motorFrontLeftPos = frontLeft.getCurrentPosition();
            int motorBackLeftPos = backLeft.getCurrentPosition();

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
                    (frontRight.isBusy() || frontLeft.isBusy())) {

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
                setPower(leftSpeed,rightSpeed);
                dilatePower(Math.abs((motorBackLeftPos + backLeft.getCurrentPosition())/(int) (dist * COUNTS)));
                drive();

            }
            this.kill();

            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}
