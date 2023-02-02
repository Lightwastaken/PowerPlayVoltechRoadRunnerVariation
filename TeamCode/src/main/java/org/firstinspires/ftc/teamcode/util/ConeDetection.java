package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cameraDetection;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfRect;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.objdetect.CascadeClassifier;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class ConeDetection extends OpenCvPipeline {
    Telemetry telemetry;
    protected Rect ROI;
    protected Mat mat = new Mat();
    CascadeClassifier coneDetect = new CascadeClassifier();
    MatOfRect detector = new MatOfRect();
    robotOffset offset;
    double xOffset;

    public ConeDetection(Telemetry t) {
        offset = robotOffset.IDLE;
        telemetry = t;
        ROI = new Rect(
                new Point(214, 0),
                new Point(319, 239));
    }

    public enum robotOffset {
        RIGHT,
        LEFT,
        ON,
        IDLE;
    }

    @Override
    public Mat processFrame(Mat input) {
        //thresholding
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = (modeNameContains("RS")) ? new Scalar(0, 0, 0) : new Scalar(100, 150, 0); //RED : BLUE
        Scalar highHSV = (modeNameContains("RS")) ? new Scalar(19, 255, 255) : new Scalar(140, 255, 255); //RED : BLUE
        Core.inRange(mat, lowHSV, highHSV, mat);

        coneDetect.detectMultiScale(mat, detector); //Detect objects in image

        //filter list of detected objects to ones whose width is less than half the ROI(minimize the chances of false cone detection)
        ArrayList<Rect> detectList = new ArrayList<Rect>((Collection<? extends Rect>) detector);
        ArrayList<Rect> detectList2 = new ArrayList<Rect>();
        for (Rect rectangle : detectList) {
            if (rectangle.width < ROI.width/2) {
                detectList2.add(rectangle);
            }
        }
        detectList.clear();


        for (Rect rect : detectList2) {
            Imgproc.rectangle(mat, new Point(rect.x, rect.y),
                    new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(0, 0, 255), 2);
        }

        Rect trackedObj = (Rect) detectList2.toArray()[0];
        if (trackedObj.x < 3 * ROI.width/4.0) {
            offset = robotOffset.LEFT;
        } else if (trackedObj.x > 3 * ROI.width/4.0) {
            offset = robotOffset.RIGHT;
        } else {
            offset = robotOffset.ON;
        }

        xOffset = (3 * ROI.width/4.0) - trackedObj.x;
        return input;
    }

    public boolean modeNameContains(String term) {
        return this.getClass().getSimpleName().contains(term);
    }

    public double getOffset() {
        return xOffset;
    }

    public double getTargetPos() {
        return 3 * ROI.width/4.0;
    }
}
