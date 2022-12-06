package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepmeeptesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36.4, 61.6, Math.toRadians(-90)))
                                .lineToSplineHeading(new Pose2d(34, 10, Math.toRadians(-130)))
                                .waitSeconds(0.5)
                                .turn(Math.toRadians(40))
                                .back(2)
                                .strafeRight(10)
                                .lineToLinearHeading(new Pose2d(55, 12, Math.toRadians(0)))
                                .waitSeconds(0.25)
                                .lineToLinearHeading(new Pose2d(24, 12, Math.toRadians(-90)))
                                .waitSeconds(0.25)
                                .strafeRight(6.5)
                                .splineToConstantHeading(new Vector2d(10, 36), Math.toRadians(90))
                                .turn(Math.toRadians(180))
                                .strafeRight(2)
                                .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}