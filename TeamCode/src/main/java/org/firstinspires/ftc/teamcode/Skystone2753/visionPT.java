package org.firstinspires.ftc.teamcode.Skystone2753;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class visionPT extends OpenCvPipeline {

    public Mat processFrame(Mat input) {
        Mat output = input;
        Point point = new Point(100,100);
        Imgproc.circle(output, point, 10, new Scalar(0,0,255));
        return output;
    }
}
