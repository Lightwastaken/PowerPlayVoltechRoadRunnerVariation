package org.firstinspires.ftc.teamcode.officialAutos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Servoconfig;
import org.firstinspires.ftc.teamcode.cameraDetection;
import org.firstinspires.ftc.teamcode.officialAutos.VISION.AprilTagAutonomousInitDetectionExample;
import org.firstinspires.ftc.teamcode.officialAutos.VISION.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.officialAutos.VISION.Vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.officialAutos.RobotHardware;
@Autonomous(name = "Blue Top + Red Bottom", group = "Pushbot")

public class BlueTop extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {
        //Measurements and stuff
        double fx = 822.317;
        double fy = 822.317;
        double cx = 319.495;
        double cy =  242.502;
        double tagsize = 0.166;

        //Initialize + start camera stream
        OpenCvCamera camera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Vision aprilTagID = new Vision(telemetry);
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
         @Override
         public void onOpened() {
             camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
         }
         @Override
         public void onError(int errorCode) {}
       });

        //Initalize hardware + program
        robot.initHW();
        waitForStart();

        //AUTO TRAJECTORIES
        Pose2d start = new Pose2d(-36.4, 61.6, 0);
        drive.setPoseEstimate(start);

        TrajectorySequence blueTop = drive.trajectorySequenceBuilder(start)
                .addDisplacementMarker(10, () -> {
                    robot.lift(0.15);
                })
                .addDisplacementMarker(63.3, () -> {
                    robot.lift(0);
                    robot.claw.setPosition((double) 255/270);
                })
                .addDisplacementMarker(65, () -> {
                    robot.lift(-0.15);
                })
                .lineToSplineHeading(new Pose2d(-35.3, 8.3, Math.toRadians(-40)))
                .waitSeconds(0.5)
                .turn(Math.toRadians(40))
                .back(2)
                .strafeLeft(10)
                .build();

        TrajectorySequence tagA = drive.trajectorySequenceBuilder(cycles.end())
                .strafeLeft(6)
                .splineToConstantHeading(new Vector2d(-12, 34), Math.toRadians(90))
                .turn(Math.toRadians(180))
                .build();

        TrajectorySequence tagB = drive.trajectorySequenceBuilder(cycles.end())
                .strafeRight(4.5)
                .splineToConstantHeading(new Vector2d(-36, 34), Math.toRadians(90))
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence tagC = drive.trajectorySequenceBuilder(cycles.end())
                .strafeRight(26)
                .splineToConstantHeading(new Vector2d(-55, 34), Math.toRadians(90))
                .turn(Math.toRadians(90))
                .back(5)
                .build();


        if(isStopRequested()) return;

        //AUTONOMOUS PATH
        drive.followTrajectorySequence(blueTop);
        cycle(3);
        if (aprilTagID.getID() == 3) {
            drive.followTrajectorySequence(tagC);
        } else if (aprilTagID.getID() == 2) {
            drive.followTrajectorySequence(tagB);
        } else {
            drive.followTrajectorySequence(tagA);
        }
    }

    TrajectorySequence cycles;
    public void cycle(int numCycles) {
        for (int i = 1; i <= numCycles; i++) {
            cycles = drive.trajectorySequenceBuilder(new Pose2d(-43.2, 15.3, -40))
                    .addDisplacementMarker(0, () -> {
                        robot.lift(-0.15);
                    })
                    .addDisplacementMarker(20 + (i/2.0), () -> {
                        robot.lift(0);
                    })
                    .addDisplacementMarker(24, () -> {
                        robot.claw.setPosition((double) 3/270);
                    })
                    .addDisplacementMarker(26, () -> {
                        robot.lift(0.1);
                    })
                    .addDisplacementMarker(88, () -> {
                        robot.lift(0);
                        robot.claw.setPosition((double) 255/270);
                    })
                    .lineToLinearHeading(new Pose2d(-53.8, 12.0, Math.toRadians(180)))
                    .waitSeconds(0.5)
                    .lineToLinearHeading(new Pose2d(-24, 12, Math.toRadians(-90)))
                    .waitSeconds(0.5)
                    .build();
        }
    }
}
