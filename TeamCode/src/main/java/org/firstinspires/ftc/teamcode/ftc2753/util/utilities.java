package org.firstinspires.ftc.teamcode.ftc2753.util;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ftc2753.subsystems.Robot;

import java.util.Locale;

public class utilities extends Robot {

    public char[] currentArray;

    public utilities() {
        currentArray = new char[50];
    }

    public char[] StringToChar(String string) {

        char[] Array = new char[string.length()];

        for (int i = 0; i < string.length(); i++) {
            Array[i] = string.charAt(i);
        }

        return Array;

    }
    public static String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private static String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public BNO055IMU returnIMU(BNO055IMU.Parameters parameters) {
        BNO055IMU imu = null;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        return imu;
    }
}
