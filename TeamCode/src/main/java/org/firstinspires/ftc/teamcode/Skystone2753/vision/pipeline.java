package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class pipeline extends OpenCvPipeline {
    public int skystonePos = 0;
    LinearOpMode linearOpMode;
    public pipeline(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
    }
    public Mat processFrame(Mat input) {
        Imgproc.medianBlur(input, input, 15);
        Imgproc.rectangle(input, new Point(40,140), new Point(100,100), new Scalar(255,0,0),3);
        Imgproc.rectangle(input, new Point(130,140), new Point(190,100), new Scalar(255,0,0),3);
        Imgproc.rectangle(input, new Point(220,140), new Point(280,100), new Scalar(255,0,0),3);

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
            linearOpMode.telemetry.addData("HELLO?", skystonePos);
            linearOpMode.telemetry.update();
            if ((Pos1a[0] + Pos1b[0] + Pos1c[0] + Pos1d[0])/4 < 100) {
                Imgproc.rectangle(input, new Point(40, 140), new Point(100, 100), new Scalar(0, 255, 0), 3);
                skystonePos = 1;
            } else if ((Pos2a[0] + Pos2b[0]+ Pos2c[0] + Pos2d[0])/4 < 100) {
                Imgproc.rectangle(input, new Point(130, 140), new Point(190, 100), new Scalar(0, 255, 0), 3);
                skystonePos = 2;
            } else if ((Pos3a[0] + Pos3b[0]+ Pos3c[0] + Pos3d[0])/4 < 100) {
                Imgproc.rectangle(input, new Point(220, 140), new Point(280, 100), new Scalar(0, 255, 0), 3);
                skystonePos = 3;
            }
        } catch (Exception e) {

        }
        return input;
    }

}


