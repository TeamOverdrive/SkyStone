package org.firstinspires.ftc.teamcode.ftc2753.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class ExtendedDriveTrain extends DriveTrain{

    private static final float  P_TURN_COEFF         = 0.1f;
    private static final double P_DRIVE_COEFF        = 0.15;
    private static final float  HEADING_THRESHOLD    =    1;

    public double speed = 0;
    public double timeout = 100;

    public void move(double inches, double speed, double timeout) {

        this.motorFrontRight.setTargetPosition(motorFrontRight.getCurrentPosition() + (int)(inches * drive.COUNTS_PER_INCH));
        this.motorBackRight.setTargetPosition(motorBackRight.getCurrentPosition() + (int)(inches * drive.COUNTS_PER_INCH));
        this.motorFrontLeft.setTargetPosition(motorFrontLeft.getCurrentPosition() + (int)(inches * drive.COUNTS_PER_INCH));
        this.motorBackLeft.setTargetPosition(motorBackLeft.getCurrentPosition() + (int)(inches * drive.COUNTS_PER_INCH));

        this.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.speed = speed;
        this.timeout = timeout;

    }
    public void strafe(String direction, double inches, double speed, double timeout) {

        if (direction.equals("LEFT")) {

            motorFrontRight.setTargetPosition(motorFrontRight.getCurrentPosition() + (int) (inches * drive.COUNTS_PER_INCH));
            motorBackRight.setTargetPosition(motorBackRight.getCurrentPosition() + (int) (-inches * drive.COUNTS_PER_INCH));
            motorFrontLeft.setTargetPosition(motorFrontLeft.getCurrentPosition() + (int) (-inches * drive.COUNTS_PER_INCH));
            motorBackLeft.setTargetPosition(motorBackLeft.getCurrentPosition() + (int) (inches * drive.COUNTS_PER_INCH));

        } else {

            motorFrontRight.setTargetPosition(motorFrontRight.getCurrentPosition() + (int) (-inches * drive.COUNTS_PER_INCH));
            motorBackRight.setTargetPosition(motorBackRight.getCurrentPosition() + (int) (inches * drive.COUNTS_PER_INCH));
            motorFrontLeft.setTargetPosition(motorFrontLeft.getCurrentPosition() + (int) (inches * drive.COUNTS_PER_INCH));
            motorBackLeft.setTargetPosition(motorBackLeft.getCurrentPosition() + (int) (-inches * drive.COUNTS_PER_INCH));

        }

        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.speed = speed;
        this.timeout = timeout;

    }
    public boolean onHeading(double speed, double angle, Orientation angles) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle,angles);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, P_TURN_COEFF);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        motorBackLeft.setPower(leftSpeed);
        motorFrontLeft.setPower(leftSpeed);
        motorBackRight.setPower(rightSpeed);
        motorFrontRight.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
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
    public double gyroAngle(double speed, double angle, Orientation angles) {
        double   error ;
        double   steer ;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle,angles);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            rightSpeed = 0.0;
        }
        else {
            steer = getSteer(error, P_TURN_COEFF);
            rightSpeed  = speed * steer;
        }

        return rightSpeed;
    }

}
