package org.firstinspires.ftc.teamcode.ftc2753.util;

public class OverdriveUtil {

    public char[] currentArray;

    public OverdriveUtil() {
        currentArray = new char[50];
    }

    public char[] StringToChar(String string) {

        char[] Array = new char[string.length()];

        for (int i = 0; i < string.length(); i++) {
            Array[i] = string.charAt(i);
        }

        return Array;

    }
}
