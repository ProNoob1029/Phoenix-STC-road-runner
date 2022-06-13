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
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 62.4, Math.toRadians(270)))
                                .splineToLinearHeading(new Pose2d(-59, 59, Math.toRadians(45)), Math.toRadians(135))
                                //.addTemporalMarker(2.6, () -> carusel.setPower(0.5))
                                //.addTemporalMarker(5.6, () -> carusel.setPower(0))
                                .waitSeconds(3)
                                .setTangent(0)
                                .splineToLinearHeading(new Pose2d(-11.67, 35, Math.toRadians(270)), Math.toRadians(270))
                                //.addTemporalMarker(7, () -> liftPosition = 1400)
                                //.addTemporalMarker(9.4, () -> cupa.setPosition(0.7))
                                //.addTemporalMarker(10.3, () -> cupa.setPosition(1))
                                //.addTemporalMarker(10.7, () -> liftPosition = 0)
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(new Vector2d(11.67, 60), Math.toRadians(0))
                                .splineTo(new Vector2d(37.5, 60), Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}