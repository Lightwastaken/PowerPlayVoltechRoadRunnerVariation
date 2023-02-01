package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cameraDetection;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ConeDetection extends OpenCvPipeline {
    Telemetry telemetry;
    protected Rect ROI;
    protected Mat mat = new Mat();
    ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    Mat hierarchy = new Mat();

    public ConeDetection(Telemetry t) {
        telemetry = t;
        ROI = new Rect(
                new Point(214, 0),
                new Point(319, 239));
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = (modeNameContains("RS")) ? new Scalar(0, 0, 0) : new Scalar(100, 150, 0); //RED : BLUE
        Scalar highHSV = (modeNameContains("RS")) ? new Scalar(19, 255, 255) : new Scalar(140, 255, 255); //RED : BLUE
        Core.inRange(mat, lowHSV, highHSV, mat);
        Mat center = mat.submat(ROI);

        double rawValue = Core.sumElems(center).val[0];
        double rawValuePercent = rawValue / ROI.area() / (255 * 100);

        Imgproc.Canny(mat, mat, 100 , 200);
        Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        center.release();
        return input;
    }

    public boolean modeNameContains(String term) {
        return this.getClass().getSimpleName().contains(term);
    }
}
