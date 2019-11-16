package org.firstinspires.ftc.teamcode.ftc2753.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveTrain {

    public double FrontLeft = 0;
    public double FrontRight = 0;
    public double BackLeft = 0;
    public double BackRight = 0;

    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorFrontRight;

    public final double COUNTS_PER_INCH = 43.465342326685739;

    public float speedMax = (float) (2 / Math.sqrt(2));

    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
    static final double HEADING_THRESHOLD = 1;


    public DriveTrain(OpMode linearOpMode) {
        initMotors(linearOpMode);
    }

    public void initMotors(OpMode linearOpMode) {

        motorBackLeft = linearOpMode.hardwareMap.get(DcMotor.class, "left_back");
        motorBackRight = linearOpMode.hardwareMap.get(DcMotor.class, "right_back");
        motorFrontLeft = linearOpMode.hardwareMap.get(DcMotor.class, "left_front");
        motorFrontRight = linearOpMode.hardwareMap.get(DcMotor.class, "right_front");

    }

    public void move(double angle, double r, double tanRight) {

        if (r > 1)
            r = 1;

        this.FrontLeft = (r * Math.cos(angle) * this.speedMax) + tanRight;
        this.FrontRight = (r * Math.sin(angle) * this.speedMax) - tanRight;
        this.BackLeft = (r * Math.sin(angle) * this.speedMax) + tanRight;
        this.BackRight = (r * Math.cos(angle) * this.speedMax) - tanRight;

    }

    public void move(String leftRight,float speed) {

        if (leftRight.equalsIgnoreCase("RIGHT")) {
            this.FrontLeft = Math.abs(speed);
            this.FrontRight = -Math.abs(speed);
            this.BackLeft = -Math.abs(speed * 0.96);
            this.BackRight = Math.abs(speed * 0.96);
        }

        if (leftRight.equalsIgnoreCase("LEFT")) {
            this.FrontLeft = -Math.abs(speed);
            this.FrontRight = Math.abs(speed);
            this.BackLeft = Math.abs(speed * 0.96);
            this.BackRight = -Math.abs(speed * 0.96);
        }
    }

    public void move(float speed) {

        this.FrontLeft = speed;
        this.FrontRight = speed;
        this.BackLeft = speed;
        this.BackRight = speed;

    }
    public void kill() {

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    public boolean onHeading(double speed, double angle, double PCoeff, Orientation angles) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle,angles);

        if (Math.abs(error) <= this.HEADING_THRESHOLD) {
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
        motorBackLeft.setPower(leftSpeed);
        motorFrontLeft.setPower(leftSpeed);
        motorBackRight.setPower(rightSpeed);
        motorFrontRight.setPower(rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle, Orientation angles) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }





}