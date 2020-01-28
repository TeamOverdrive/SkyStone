// Made by Timothy *before qualification*
package org.firstinspires.ftc.teamcode.Skystone2753.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

// instantiate Vision object to detect skystones. Read skystone location with
// vision.pipeline.skystonePos <-- returns int (1 = LEFT, 2 = "MID", 3 = "RIGHT", 0 = "VOID")
public class Vision {

    // init camera and necessary variables
    OpenCvCamera webcam;
    int cameraMonitorViewId;

    // pipeline has internal skystonePos that is updated based on skystone detected inside of
    // pipeline. External code that instantiates Vision can access internal skystonePos with
    // vision.pipeline.skystonePos <-- returns int
    public pipeline pipeline;

    // LinearOpMode required to use hardwareMap
    LinearOpMode linearOpMode;

    // pass in opMode using "this", Vision can then use opMode's hardwareMap to get camera
    public Vision(LinearOpMode linearOpMode) {

        this.linearOpMode = linearOpMode;

        // init camera
        cameraMonitorViewId = linearOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", linearOpMode.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(linearOpMode.hardwareMap.get(WebcamName.class, "Webcam8"), cameraMonitorViewId);
        webcam.openCameraDevice();

        // pipeline needs to be init with linearOpMode for skystone position telemetry output
        pipeline = new pipeline(linearOpMode);
        webcam.setPipeline(pipeline);

        // startStreaming tends to throw a exception the first time it is run. Try & Catch
        // statement used to avoid this.
        try {
            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        } catch (Exception e) {
            linearOpMode.telemetry.addLine("Camera unable to start steam...");
            linearOpMode.telemetry.addLine("Please wait...");
            linearOpMode.telemetry.update();
        }

    }

}
