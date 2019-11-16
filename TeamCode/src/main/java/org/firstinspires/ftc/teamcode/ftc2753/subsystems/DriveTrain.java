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

    public final double COUNTS_PER_INCH = 43.465342326685739;

    public float speedMax = (float) (2 / Math.sqrt(2));

    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
    static final double HEADING_THRESHOLD = 1;


    public DriveTrain() {
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