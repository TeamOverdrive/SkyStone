package org.firstinspires.ftc.teamcode.Skystone2753;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "TESTING", group = "Does this even matter?")
public class SandboxAuto extends LinearOpMode {

    Robot robot = new Robot(this);
    public void runOpMode() {
        robot.getDrive().moveDist(24,1,this);
    }
}
