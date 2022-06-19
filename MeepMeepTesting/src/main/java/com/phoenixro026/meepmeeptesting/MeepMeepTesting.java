package com.phoenixro026.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(
                        28.581263557573042,
                        28.581263557573042,
                        Math.toRadians(172.37744999999998),
                        Math.toRadians(172.37744999999998),
                        8.92
                )
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -61.7, Math.toRadians(90)))
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-11.67, 35.6, Math.toRadians(270)), Math.toRadians(270))
                                //.addTemporalMarker(3.5, () -> cupa.setPosition(0.75))
                                //.addTemporalMarker(4.5, () -> cupa.setPosition(1))
                                //.addTemporalMarker(5, () -> liftPosition = 0)
                                .waitSeconds(2)
                                .setReversed(true)
                                .splineTo(new Vector2d(11.67, 61.5), Math.toRadians(0))
                                .splineTo(new Vector2d(29.5, 61.5), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(41, 32), Math.toRadians(270))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot1)
                //.addEntity(myBot2)
                //.addEntity(myBot3)
                //.addEntity(myBot4)
                .start();
    }
}