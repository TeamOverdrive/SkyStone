package org.firstinspires.ftc.teamcode.ftc2753.vision;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.HashMap;

import org.opencv.core.*;
import org.opencv.core.Core.*;
import org.opencv.features2d.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;
import org.openftc.easyopencv.OpenCvPipeline;

public class StoneDetector extends OpenCvPipeline {

    //Outputs
    private Mat hslThresholdOutput = new Mat();

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }


    public void process(Mat source0) {
        // Step HSL_Threshold0:
        Mat hslThresholdInput = source0;
        double[] hslThresholdHue = {0.0, 180.0};
        double[] hslThresholdSaturation = {155.93525179856115, 255.0};
        double[] hslThresholdLuminance = {43.57014388489208, 255.0};
        hslThreshold(hslThresholdInput, hslThresholdHue, hslThresholdSaturation, hslThresholdLuminance, hslThresholdOutput);

    }

    public Mat hslThresholdOutput() {
        return hslThresholdOutput;
    }


    private void hslThreshold(Mat input, double[] hue, double[] sat, double[] lum,
                              Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HLS);
        Core.inRange(out, new Scalar(hue[0], lum[0], sat[0]),
                new Scalar(hue[1], lum[1], sat[1]), out);
    }


    @Override
    public Mat processFrame(Mat input) {
        Imgproc.rectangle(
                input,
                new Point(
                        input.cols()/4,
                        input.rows()/4),
                new Point(
                        input.cols()*(3f/4f),
                        input.rows()*(3f/4f)),
                new Scalar(0, 255, 0), 4);

        return input;
    }
}

