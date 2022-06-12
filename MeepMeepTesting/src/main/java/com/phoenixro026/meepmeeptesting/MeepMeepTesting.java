package com.phoenixro026.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(
                        28.581263557573042,
                        28.581263557573042,
                        Math.toRadians(172.37744999999998),
                        Math.toRadians(172.37744999999998),
                        8.92
                )
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.5, -62.4, Math.toRadians(90)))
                                .splineTo(new Vector2d(-6.5, -38.5), Math.toRadians(110))
                                .forward(6)
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(new Vector2d(11.5, -62.4), Math.toRadians(0))
                                .back(30)
                                .strafeRight(22)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}