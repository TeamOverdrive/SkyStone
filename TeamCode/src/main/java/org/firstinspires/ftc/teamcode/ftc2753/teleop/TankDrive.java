package org.firstinspires.ftc.teamcode.ftc2753.teleop;

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TankDrive")
public class TankDrive extends LinearOpMode {

    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;

    boolean soundPlaying = false;

    private boolean bruhFound;

    private boolean isRB = false;    // Gamepad button state variables
    private boolean wasRB = false;    // Gamepad button state variables

    @Override
    public void runOpMode() throws InterruptedException {

        int soundBruhID   = hardwareMap.appContext.getResources().getIdentifier("bruh",   "raw", hardwareMap.appContext.getPackageName());
        if (soundBruhID != 0)
            bruhFound   = SoundPlayer.getInstance().preload(hardwareMap.appContext, soundBruhID);

        telemetry.addData("bruh.wav",   bruhFound ?   "Found" : "NOT found\n Add bruh.wav to /src/main/res/raw" );

        motorBackLeft = hardwareMap.dcMotor.get("left1");
        motorBackRight = hardwareMap.dcMotor.get("right1");
        motorFrontLeft = hardwareMap.dcMotor.get("left2");
        motorFrontRight = hardwareMap.dcMotor.get("right2");

        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);

        Context myApp = hardwareMap.appContext;

        waitForStart();

            while (opModeIsActive()) {

                motorBackLeft.setPower(-gamepad1.left_stick_y);
                motorFrontLeft.setPower(-gamepad1.left_stick_y);
                motorBackRight.setPower(-gamepad1.right_stick_y);
                motorFrontRight.setPower(-gamepad1.right_stick_y);

                if (bruhFound && (isRB = gamepad1.right_bumper) && !wasRB) {
                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundBruhID);
                    telemetry.addData("Playing", "Resource Silver");
                    telemetry.update();
                }

                wasRB = isRB;

            }

        }

    }
