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
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(90)))
                                .strafeRight(25)
                                .lineToLinearHeading(new Pose2d(-11, -16, Math.toRadians(128)))
                                .forward(10)
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(-11, -13, Math.toRadians(90)))
                                .strafeLeft(12)
                                .lineToSplineHeading(new Pose2d(-57, -11.5, Math.toRadians(180)))
                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(-24, -12, Math.toRadians(90)))
                                .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}