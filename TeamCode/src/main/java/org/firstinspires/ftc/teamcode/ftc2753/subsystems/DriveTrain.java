package org.firstinspires.ftc.teamcode.ftc2753.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveTrain extends Robot{

    public double FrontLeft, FrontRight, BackLeft, BackRight;
    public DcMotor motorBackLeft, motorBackRight, motorFrontLeft, motorFrontRight;

    public double[] powers = {FrontLeft, FrontRight, BackLeft, BackRight};
    public DcMotor[] motors = {motorBackLeft, motorBackRight, motorFrontLeft, motorFrontRight};

    public final double COUNTS_PER_INCH = 43.465342326685739;
    public final float speedMax = (float) (2 / Math.sqrt(2));
    public float speed = 1, turnSpeed = 1;


    public DriveTrain() {

    }

    public void init() {

        motorBackLeft = hardwareMap.get(DcMotor.class, "left_back");
        motorBackRight = hardwareMap.get(DcMotor.class, "right_back");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "left_front");
        motorFrontRight = hardwareMap.get(DcMotor.class, "right_front");

        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);

    }
    public void move(double angle, double speed, double turn) {

        setSpeed((speed * Math.cos(angle) * this.speedMax) + (turn * turnSpeed),(speed * Math.sin(angle) * this.speedMax) - (turn * turnSpeed),
                (speed * Math.sin(angle) * this.speedMax) + (turn * turnSpeed),(speed * Math.cos(angle) * this.speedMax) - (turn * turnSpeed));
    }

    public void move(String leftRight,float speed) {

        double velocity = Math.abs(speed);
        if (leftRight.equalsIgnoreCase("RIGHT")) {
            setSpeed(velocity,-velocity,-velocity,velocity);
        }

        if (leftRight.equalsIgnoreCase("LEFT")) {
            setSpeed(-velocity,velocity,velocity,-velocity);
        }
    }

    public void move(float speed) {

        for (int i = 0; i < this.powers.length; i++) {
            this.powers[i] = speed;
        }
    }
    public void setSpeed(double FL, double FR, double BL, double BR) {
        FrontLeft = FL;
        FrontRight = FR;
        BackLeft = BL;
        BackRight = BR;
    }
    public void update() {
        for (int i = 0; i < motors.length;i++) {
            motors[i].setPower(powers[i] * speed);
        }
    }
    public void kill() {
        for (int i = 0; i < motors.length;i++) {
            motors[i].setPower(0);
        }
    }
    public void teleDrive(Orientation angles) {
        double relativeAngle;
        relativeAngle = (Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4) - Math.toRadians(angles.firstAngle);
        if (Math.abs(relativeAngle) > Math.PI) {
            if (relativeAngle > 0)
                relativeAngle = -(Math.PI * 2 - Math.abs(relativeAngle));
            else if (relativeAngle > 0)
                relativeAngle = Math.PI * 2 - Math.abs(relativeAngle);
        }

        move(relativeAngle, Math.sqrt(gamepad1.left_stick_x * gamepad1.left_stick_x +  gamepad1.left_stick_y * gamepad1.left_stick_y) * speed,
                gamepad1.right_stick_x * turnSpeed);
    }
    public void setBrake(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setBrake() {
        for (int i = 0; i < motors.length;i++) {
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    public void removeBrake(DcMotor motor) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void removeBrake() {
        for (int i = 0; i < motors.length;i++) {
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

}