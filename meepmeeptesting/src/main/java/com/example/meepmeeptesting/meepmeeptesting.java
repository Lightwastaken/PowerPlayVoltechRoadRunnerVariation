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
                .setConstraints(52.48180821614297, 52.48180821614297, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(-90)))
                                .forward(1.5)
                                .strafeLeft(24)
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(-11, 15, Math.toRadians(-128)))
                                .forward(9)
                                .waitSeconds(0.25)
                                .waitSeconds(0.5)
                                .waitSeconds(0.75)
                                .back(5)
                                .lineToLinearHeading(new Pose2d(-24, 12, Math.toRadians(-90)))

                                .lineToLinearHeading(new Pose2d(-57, 12, Math.toRadians(-180)))
                                .waitSeconds(1.6325)
                                .forward(1.25)
                                .waitSeconds(1)
                                .waitSeconds(0.75)
                                .lineToLinearHeading(new Pose2d(-24, 9, Math.toRadians(-90)))
                                .waitSeconds(0.5)
                                .waitSeconds(0.75)
                                .waitSeconds(1)
                                .back(3)
                                .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}