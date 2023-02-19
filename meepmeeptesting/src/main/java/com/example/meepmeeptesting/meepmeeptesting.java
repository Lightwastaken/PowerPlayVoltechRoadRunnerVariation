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
                .setConstraints(52.48180821614297, 52.48180821614297, Math.toRadians(180), Math.toRadians(180), 24)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, 60, Math.toRadians(180)))
                                .strafeLeft(50)
                                .splineToLinearHeading(new Pose2d(-32, 10, Math.toRadians(135)), Math.toRadians(135))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(-57, 12, Math.toRadians(180)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(-32, 10, Math.toRadians(135)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(-57, 12, Math.toRadians(180)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(-32, 10, Math.toRadians(135)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(-57, 12, Math.toRadians(180)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(-32, 10, Math.toRadians(135)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(-57, 12, Math.toRadians(180)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(-32, 10, Math.toRadians(135)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(-57, 12, Math.toRadians(180)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(-32, 10, Math.toRadians(135)))
                                .waitSeconds(0.5)
                                .turn(Math.toRadians(-45))
                                .build()

                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}