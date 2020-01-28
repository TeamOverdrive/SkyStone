// Made by Timothy *before qualification*
package org.firstinspires.ftc.teamcode.Skystone2753.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

// used by Vision.java to get skystonePos. Camera is created in Vision.
public class pipeline extends OpenCvPipeline {

    // if not skystone is ever detected the skystonePos defaults to 0
    public int skystonePos = 0;

    // LinearOpMode needed for telemetry
    LinearOpMode linearOpMode;
    public pipeline(LinearOpMode linearOpMode) {

        this.linearOpMode = linearOpMode;

    }

    public Mat processFrame(Mat input) {

        // blur image to remove odd colored pixels (white outliers)
        Imgproc.medianBlur(input, input, 15);

        // create red rectangles around positions being checked
        Imgproc.rectangle(input, new Point(40,140), new Point(100,100), new Scalar(255,0,0),3);
        Imgproc.rectangle(input, new Point(130,140), new Point(190,100), new Scalar(255,0,0),3);
        Imgproc.rectangle(input, new Point(220,140), new Point(280,100), new Scalar(255,0,0),3);

        // pixels hardcoded to be checked, receives rgb in double[]
        // TODO: create function to check & average all pixels in rectangle
        double[] Pos1a = input.get(125,75);
        double[] Pos2a = input.get(125,165);
        double[] Pos3a = input.get(125,255);
        double[] Pos1b = input.get(120,70);
        double[] Pos2b = input.get(120,160);
        double[] Pos3b = input.get(120,250);
        double[] Pos1c = input.get(115,65);
        double[] Pos2c = input.get(115,155);
        double[] Pos3c = input.get(115,245);
        double[] Pos1d = input.get(125,70);
        double[] Pos2d = input.get(125,160);
        double[] Pos3d = input.get(125,250);

        try {

            linearOpMode.telemetry.addLine("Camera initialized! Please check camera rotation.");

            // check if red value is less than 100 (in 0-255 range)
            if ((Pos1a[0] + Pos1b[0] + Pos1c[0] + Pos1d[0])/4 < 100) {
                // draw green rectangle over red one
                Imgproc.rectangle(input, new Point(40, 140), new Point(100, 100), new Scalar(0, 255, 0), 3);
                skystonePos = 1;
                linearOpMode.telemetry.addLine("Skystone found: RIGHT POS");
                linearOpMode.telemetry.update();

            } else if ((Pos2a[0] + Pos2b[0]+ Pos2c[0] + Pos2d[0])/4 < 100) {
                Imgproc.rectangle(input, new Point(130, 140), new Point(190, 100), new Scalar(0, 255, 0), 3);
                skystonePos = 2;
                linearOpMode.telemetry.addLine("Skystone found: RIGHT POS");
                linearOpMode.telemetry.update();

            } else if ((Pos3a[0] + Pos3b[0]+ Pos3c[0] + Pos3d[0])/4 < 100) {
                Imgproc.rectangle(input, new Point(220, 140), new Point(280, 100), new Scalar(0, 255, 0), 3);
                skystonePos = 3;
                linearOpMode.telemetry.addLine("Skystone found: RIGHT POS");
                linearOpMode.telemetry.update();

            }
        // camera sometimes glitches and returns null rgb values
        } catch (Exception e) {

        }

        // return input image Matrix but with the rectangles
        return input;
    }

}


