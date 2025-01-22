package org.firstinspires.ftc.teamcode.vision.tuning;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.opencv.core.Core.inRange;
import static org.opencv.imgproc.Imgproc.*;
import static org.opencv.imgproc.Imgproc.drawContours;

public class FilterTuner extends OpenCvPipeline {

    Telemetry telemetry;
    public FilterTuner(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    enum DisplayMode {
        RAW_CAMERA_INPUT,
        RAW_MASK,
        DENOISED_MASK,
        CONTOURS
    }

    private DisplayMode displayMode = DisplayMode.CONTOURS;

    //red
    public Scalar lower = new Scalar(0.0, 0.0, 0.0);
    public Scalar upper = new Scalar(255.0, 255.0, 255.0);


    //blue
    //public Scalar lower = new Scalar(0.0, 110.0, 150.0);
    //public Scalar upper = new Scalar(255.0, 150.0, 200.0);

    @Override
    public Mat processFrame(Mat input) {
        Mat realInput = input.clone();
        Mat rawInput = input.clone();

        //colorspace
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2Lab);

        //filter
        inRange(input, lower, upper, input);
        Mat rawMask = input.clone();

        //denoise
        Imgproc.morphologyEx(input, input, Imgproc.MORPH_CLOSE, Mat.ones(5,5, CvType.CV_32F));
        Imgproc.morphologyEx(input, input, Imgproc.MORPH_OPEN, Mat.ones(3,3, CvType.CV_32F));
        Mat denoisedMask = input.clone();

        //find and draw contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        findContours(input, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        drawContours(realInput, contours,-1, new Scalar(255, 0, 0), 2);

        //return realInput;

        switch (displayMode) {
            case RAW_CAMERA_INPUT:
                return rawInput;
            case RAW_MASK:
                return rawMask;
            case DENOISED_MASK:
                return denoisedMask;
            case CONTOURS:
                return realInput;
            default:
                return realInput;
        }
    }
    @Override
    public void onViewportTapped() {
        DisplayMode[] modes = DisplayMode.values();
        int nextOrdinal = (displayMode.ordinal() + 1) % modes.length;
        displayMode = modes[nextOrdinal];
    }

}
