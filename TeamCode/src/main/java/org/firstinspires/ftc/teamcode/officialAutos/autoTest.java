package org.firstinspires.ftc.teamcode.officialAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous

public class autoTest extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initHW();
        drive.setPoseEstimate(new Pose2d());
        waitForStart();
        Pose2d myPose = new Pose2d(10, -5, Math.toRadians(90));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory blueTop = drive.trajectorySequenceBuilder(new Pose2d())
                .waitSeconds(5)
                .forward(36)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(blueTop);    }
}
