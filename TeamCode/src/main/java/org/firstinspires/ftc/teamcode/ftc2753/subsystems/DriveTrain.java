package org.firstinspires.ftc.teamcode.ftc2753.subsystems;

public class DriveTrain {

    public double FrontLeft = 0;
    public double FrontRight = 0;
    public double BackLeft = 0;
    public double BackRight = 0;

    public final double COUNTS_PER_INCH = 43.465342326685739;

    public float speedMax = (float) (2 / Math.sqrt(2));


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
            this.BackLeft = -Math.abs(speed * 0.87);
            this.BackRight = Math.abs(speed * 0.87);
        }

        if (leftRight.equalsIgnoreCase("LEFT")) {
            this.FrontLeft = -Math.abs(speed);
            this.FrontRight = Math.abs(speed);
            this.BackLeft = Math.abs(speed * 0.87);
            this.BackRight = -Math.abs(speed * 0.87);
        }
    }

    public void move(float speed) {

        this.FrontLeft = speed;
        this.FrontRight = speed;
        this.BackLeft = speed;
        this.BackRight = speed;

    }

}