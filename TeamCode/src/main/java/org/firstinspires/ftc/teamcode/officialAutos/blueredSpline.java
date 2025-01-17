/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.officialAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.VISION.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;
import java.util.ArrayList;


@Disabled
@Autonomous(name="SplineOp(BR)", group="Pushbot")
public class blueredSpline extends LinearOpMode {
    public static Pose2d preloadEnd;
    public static Pose2d cycleEnd;
    double[] stack = {500, 475, 450, 425, 400};
    int n = 0;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 822.317;
    double fy = 822.317;
    double cx = 319.495;
    double cy =  242.502;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 18 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotHardware robot = new RobotHardware(this);
        Pose2d start = new Pose2d(36, -60, Math.toRadians(90));
        drive.setPoseEstimate(start);

        telemetry.setMsTransmissionInterval(50);
        robot.initHW();
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT||tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        TrajectorySequence preloadDeliver = drive.trajectorySequenceBuilder(start)
                .strafeLeft(5)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.lift(robot.PIDControl(robot.TOP_OUTTAKE_POSITION, robot.RTL.getCurrentPosition()));
                })
                .lineToLinearHeading(new Pose2d(-36, 10, Math.toRadians(-54)))
                .waitSeconds(0.1)
                .forward(6)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.lift(robot.PIDControl(25, robot.RTL.getCurrentPosition()));
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.claw.setPosition(0.1);
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-24, 10, Math.toRadians(-90)))
//                .lineToLinearHeading(new Pose2d(-60.4, 9, Math.toRadians(-175)))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.lift(robot.PIDControl(500, robot.RTL.getCurrentPosition()));
//                    robot.lift(robot.PIDControl(500, robot.RTL.getCurrentPosition()));
//                    robot.lift(robot.PIDControl(500, robot.RTL.getCurrentPosition()));
//                })
//                .forward(1.25)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.claw.setPosition(1);
//                })
//                .waitSeconds(1)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> { robot.lift(robot.PIDControl(RobotHardware.TOP_OUTTAKE_POSITION, robot.RTL.getCurrentPosition())); })
//                .waitSeconds(0.75)
//                .lineToLinearHeading(new Pose2d(-24, 6, Math.toRadians(-90)))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> { robot.lift(0); })
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.lift(robot.PIDControl(RobotHardware.BOTTOM_OUTTAKE_POSITION, robot.RTL.getCurrentPosition()));
//                })
//                .waitSeconds(0.75)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.claw.setPosition(0.1);
//                })
//                .waitSeconds(1)
//                .back(3)
                .build();

        preloadEnd = preloadDeliver.end();


        TrajectorySequence leftTOI = drive.trajectorySequenceBuilder(preloadEnd)
                .strafeLeft(13)
                .build();

        TrajectorySequence middleTOI = drive.trajectorySequenceBuilder(preloadEnd)
                .strafeRight(13)
                .build();

        TrajectorySequence rightTOI = drive.trajectorySequenceBuilder(preloadEnd)
                .back(0.25)
                .strafeRight(42)
                .build();


        //TRAJECTORY FOLLOWED
        drive.followTrajectorySequence(preloadDeliver);
        if (tagOfInterest == null || tagOfInterest.id == LEFT) { //LEFT parking: ID #1
            drive.followTrajectorySequence(leftTOI);
        } else if (tagOfInterest.id == MIDDLE) { //MIDDLE parking: ID #2
            drive.followTrajectorySequence(middleTOI);
        } else { //RIGHT parking; ID #3
            drive.followTrajectorySequence(rightTOI);
        }



        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public void cycles(int numCycles, SampleMecanumDrive drive, RobotHardware robot) {
        TrajectorySequence cycle = drive.trajectorySequenceBuilder(preloadEnd)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.lift(robot.PIDControl(stack[n], robot.RTL.getCurrentPosition()));
                })
                .lineToLinearHeading(new Pose2d(-60, 9, Math.toRadians(180.4)))
                .waitSeconds(0.25)
                .forward(4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.claw.setPosition(1);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.lift(robot.PIDControl(RobotHardware.TOP_OUTTAKE_POSITION, robot.RTL.getCurrentPosition()));
                })
                .lineToLinearHeading(preloadEnd)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.lift(-0.1);
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.claw.setPosition(0.1);
                })
                .waitSeconds(0.25)
                .build();

        for (int i = 0; i < numCycles; i++) {
            drive.followTrajectorySequence(cycle);
        }

        cycleEnd = cycle.end();
    }
}