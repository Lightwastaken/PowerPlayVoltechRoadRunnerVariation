package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
/*
Code explanation:
1) Region of Interest created to detect signal sleeve colors
2) Camera stream converted from RGB to HSV and masks out colors(those become black) except those defined within bounds(orange-brown color, those become white)
3) Program calculates ratio of area of white to area of black
4) If-else if-else statement to check the ratio against certain thresholds to determine what color the sleeve is
5) Program returns which parking position robot needs to go to and changes camera stream border to a certain color
 */

public class cameraDetection extends OpenCvPipeline {
    Telemetry telemetry;
    protected Rect ROI;
    protected Mat dMat = new Mat();
    protected Mat mat = new Mat();

    //Parking position possibilities + unknown
    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT,
        UNKNOWN
    }

    //Parking position initially set to unknown
    private ParkingPosition location = ParkingPosition.UNKNOWN;

    public cameraDetection(Telemetry t) {
        telemetry = t;
        //Create ROI camera will stream
        ROI = new Rect(
                new Point(214, 0),
                new Point(319, 239));
    }

    @Override
    public Mat processFrame(Mat input) {
        //Changes RGB --> HSV, masks image to only show a certain color range
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        //HSV bounds are for brown-orange
        //https://stackoverflow.com/questions/10948589/choosing-the-correct-upper-and-lower-hsv-boundaries-for-color-detection-withcv
        Scalar lowHSV = new Scalar(15, 100, 20);
        Scalar highHSV = new Scalar(25, 255, 255);
        Core.inRange(mat, lowHSV, highHSV, dMat);
        mat = dMat;

        //Find ROI's total value and percentage(aka find amount of white in the image after image was masked) + debugging help
        Mat center = mat.submat(ROI);
        double rawValue = Core.sumElems(center).val[0];
        double rawValuePercent = rawValue / ROI.area() / (255 * 100);
        center.release();
        telemetry.addData("Raw value: ", rawValue);
        telemetry.addData("Raw value as a percentage: ", Math.round(rawValuePercent * 100) + "%" );

        //Compare rawValuePercent to certain thresholds to determine which color the sleeve is
        Scalar resultColor;
        if (rawValuePercent > 0.8) { //ORANGE, DICE ROLL = 2 OR 5
            location = ParkingPosition.CENTER;
            resultColor = new Scalar(255, 0, 0);
        } else if (rawValuePercent > 0.4) { //BROWN, DICE ROLL = 3 OR 6
            location = ParkingPosition.RIGHT;
            resultColor = new Scalar(0, 255, 0);
        } else { //GREEN, DICE ROLL = 1 OR 4
            location = ParkingPosition.LEFT;
            resultColor = new Scalar(0, 0, 255);
        }

        //Returns parking location
        telemetry.addData("Parking location: ", location.toString());
        telemetry.update();

        //Change image back to RGB, changes border color of camera stream depending on parking position determined
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Imgproc.rectangle(mat, ROI, resultColor);
        return input;
    }

    public ParkingPosition getLocation() {
        return location;
    }

}