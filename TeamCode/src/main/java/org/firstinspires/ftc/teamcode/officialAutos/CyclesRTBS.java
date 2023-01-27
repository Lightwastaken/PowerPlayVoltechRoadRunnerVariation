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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.VISION.Vision;
import org.firstinspires.ftc.teamcode.officialAutos.VISION.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Red terminal blue substation", group="Pushbot")
public class CyclesRTBS extends LinearOpMode{
    public static Pose2d preloadEnd;
    public static Pose2d cycleEnd;
    public DistanceSensor sensor;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public enum robotState { //FSM state initialization
        PRELOAD,
        CYCLES,
        PARKING,
        IDLE
    }
    robotState currentState = robotState.IDLE;

    @Override
    public void runOpMode() {
        //ROBOT declarations
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotHardware robot = new RobotHardware(this);

        //VISION initialization
        Vision cam = new Vision(telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            public void onOpened() { camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT); }

            public void onError(int errorCode) {}
        });


        //ROBOT + trajectory initializations
        Pose2d start = new Pose2d(36, 60, Math.toRadians(-90));
        drive.setPoseEstimate(start);
        robot.initHW();

        //TRAJECTORIES
        TrajectorySequence preloadDeliver = drive.trajectorySequenceBuilder(start)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.claw.setPosition(1);
                    robot.liftEncoderDrive(0.02, 35, 35);
                })
                .forward(41.5)
                .splineToConstantHeading(new Vector2d(24, 11), Math.toRadians(-90))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.liftEncoderDrive(-0.01, 5, 5);
                    robot.claw.setPosition(0.1);
                })
                .waitSeconds(0.75)
                .build();

        preloadEnd = preloadDeliver.end();

        TrajectorySequence leftTOI = drive.trajectorySequenceBuilder(cycleEnd)
                .strafeLeft(13)
                .build();

        TrajectorySequence middleTOI = drive.trajectorySequenceBuilder(cycleEnd)
                .back(0.25)
                .strafeRight(13)
                .build();

        TrajectorySequence rightTOI = drive.trajectorySequenceBuilder(cycleEnd)
                .back(0.25)
                .strafeRight(38)
                .build();

        //TRAJECTORY FOLLOWED
        currentState = robotState.PRELOAD;
        switch (currentState) {
            case PRELOAD:
                drive.followTrajectorySequenceAsync(preloadDeliver);
                currentState = robotState.CYCLES;
                break;
            case CYCLES:
                cycles(0, drive, robot);
                currentState = robotState.PARKING;
                break;
            case PARKING:
                if (cam.getID() == 2) { //MIDDLE parking: ID #2
                    drive.followTrajectorySequenceAsync(middleTOI);
                } else if (cam.getID() == 3) { //MIDDLE parking: ID #3
                    drive.followTrajectorySequenceAsync(rightTOI);
                } else { //LEFT parking; ID #1
                    drive.followTrajectorySequenceAsync(leftTOI);
                }
                currentState = robotState.IDLE;
                break;
            case IDLE:
                break;
        }

        while (robot.isChassisVeloZero() && currentState == robotState.CYCLES) {
            getDistance();
        }


    }

    public void cycles(int numCycles, SampleMecanumDrive drive, RobotHardware robot) {
        ElapsedTime time = new ElapsedTime();
        TrajectorySequence cycle = drive.trajectorySequenceBuilder(preloadEnd)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.liftEncoderDrive(-0.01, 32, 32);
                })
                .forward(3)
                .lineToLinearHeading(new Pose2d(56.5, 11.75, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    while (getDistance() < 5) {
                        robot.lift(0.05);
                    }
                    robot.lift(0);
                })
                .waitSeconds(1.7)
                .forward(3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.claw.setPosition(1);
                    robot.liftEncoderDrive(0.05, 35, 35);
                })
                .waitSeconds(0.75)
                .lineToLinearHeading(preloadEnd)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.liftEncoderDrive(-0.05, 3, 3);
                    robot.claw.setPosition(0.1);
                })
                .build();

        for (int i = 0; i < numCycles; i++) {
            if (time.time() < 24) {
                drive.followTrajectorySequenceAsync(cycle);
            } else {
                i = numCycles + 1;
            }
        }
        cycleEnd = cycle.end();
    }

    public double getDistance() {
        double distance = sensor.getDistance(DistanceUnit.CM);
        return distance;
    }
}