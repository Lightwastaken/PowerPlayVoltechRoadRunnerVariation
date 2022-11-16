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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.officialAutos.RobotHardware;
@Autonomous

public class BlueTop extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initHW();
        waitForStart();

        Pose2d start = new Pose2d(-36.4, 61.6, 0);
        drive.setPoseEstimate(start);

        Trajectory blueTop = drive.trajectoryBuilder(start)
                .lineToSplineHeading(new Pose2d(-35.3, 8.3, Math.toRadians(-40)))
                .back(10)
                .build();

        Trajectory tagA = drive.trajectoryBuilder(cycles.end())
                .strafeRight(36)
                .waitSeconds(3)
                .back(20)
                .build();

        Trajectory tagB = drive.trajectoryBuilder(cycles.end())
                .strafeRight(12)
                .back(20)
                .build();

        Trajectory tagC = drive.trajectoryBuilder(cycles.end())
                .strafeLeft(12)
                .back(20)
                .build();
        Trajectory wait = drive.trajectoryBuilder(cycles.end())
                .build();



        waitForStart();

        if(isStopRequested()) return;
        drive.followTrajectory(blueTop);



    }

    Trajectory cycles;
    public void cycle(int numCycles) {
        for (int i = 1; i <= numCycles; i++) {
            cycles = drive.trajectoryBuilder(new Pose2d(-43.2, 15.3, -40))
                    .lineToLinearHeading(new Pose2d(-53.8, 12.0, Math.toRadians(180)))
                    .waitSeconds(0.5)
                    .lineToLinearHeading(new Pose2d(-24, 12, Math.toRadians(-90)))
                    .waitSeconds(0.5)
                    .build();
        }
    }
}
