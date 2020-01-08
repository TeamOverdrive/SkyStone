package org.firstinspires.ftc.teamcode.Skystone2753;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class Team2753LinearOpMode extends LinearOpMode {

    public Team2753LinearOpMode() {

    }
    protected void sleep(int milliseconds) {
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.milliseconds() < milliseconds && opModeIsActive()) {

        }
    }
}
