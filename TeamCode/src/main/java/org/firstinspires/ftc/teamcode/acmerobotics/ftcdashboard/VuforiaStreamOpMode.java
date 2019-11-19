package org.firstinspires.ftc.teamcode.acmerobotics.ftcdashboard;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;

/*
 * This sample demonstrates how to stream frames from Vuforia to the dashboard. Make sure to fill in
 * your Vuforia key below and select the 'Camera' preset on top right of the dashboard. This sample
 * also works for UVCs with slight adjustments.
 */
@Autonomous
@Disabled
public class VuforiaStreamOpMode extends LinearOpMode {


    public static final String VUFORIA_LICENSE_KEY =
            "AasMRnb/////AAABmfnajLJrOEyppDcz2Bh0W2Am2zcR8ujLMs+DIkcviBzeETt1IDXmO9i6rZfti6VsQ008860DREQXS2eiTg5gTbQ9XN0zoLA/c0qsFccWTA+429o3ZyJqDddgdy4FlGDGk+YDsE6nqTSSr3fVDmS5lAZ+3rBEUQ3ksutkZMuNQigcjVH1DPqLFsZWpSCTmyVvfMuu4Va+xEXloMdm0eza0a1xWAj7HZ6uTZiQS4cL+tCFy3o8pQCdpxTqsWMscq9tn//ADCGqz6jTpR5BTiCEx6azarjeL8KI/S608mjxUhUx168yttZOeK9Bo3INNA2D/wGJn/r5EWewYoxcqTAoD/MF8a2ip/Fhvq/RXn7dYXA8";

    @Override
    public void runOpMode() throws InterruptedException {
        // gives Vuforia more time to exit before the watchdog notices
        msStuckDetectStop = 2500;

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        waitForStart();

        while (opModeIsActive());
    }
}