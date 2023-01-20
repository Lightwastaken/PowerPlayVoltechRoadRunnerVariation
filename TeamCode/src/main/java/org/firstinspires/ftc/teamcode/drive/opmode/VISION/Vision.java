package org.firstinspires.ftc.teamcode.drive.opmode.VISION;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.officialAutos.VISION.AprilTagDetectionPipeline;

import java.util.ArrayList;

public class Vision extends OpenCvPipeline{
    //Variable declarations
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    AprilTagDetection tagOfInterest;
    Telemetry telemetry;
    String robotParksAt;

    //AprilTag IDs. 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    public Vision(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        if (tagOfInterest.id == 3) { //RANDOMIZATION = 3, 6
            robotParksAt = "RIGHT";
        } else if (tagOfInterest.id == 2) { //RANDOMIZATION = 2, 5
            robotParksAt = "MIDDLE";
        } else { //RANDOMIZATION = 1, 4
            robotParksAt = "LEFT";
        }

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if (currentDetections.size() != 0) {
            boolean tagFound = false;
            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if (tagFound) {
                telemetry.addData("Tag ID: ", tagOfInterest.id);
                telemetry.addData("Robot parking location: ", robotParksAt);
            } else {
                telemetry.addLine("Tag not found");
            }

        } else {
            telemetry.addLine("--- Tag not found, but last seen at: ---");
            telemetry.addData("Detected tag ID: ", tagOfInterest.id);
            telemetry.addData("Robot parking location: ", robotParksAt);
        }

        telemetry.update();

        return input;
    }

    public int getID() {
        return tagOfInterest.id;
    }

}