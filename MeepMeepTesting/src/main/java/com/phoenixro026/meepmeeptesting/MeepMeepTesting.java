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
                                .setTangent(Math.toRadians(135))
                                .splineToLinearHeading(new Pose2d(-58, -58, Math.toRadians(135)), Math.toRadians(225))
                                //.addTemporalMarker(2.3, () -> carusel.setPower(0.4))
                                //.addTemporalMarker(5.3, () -> carusel.setPower(0))
                                .waitSeconds(3)
                                .setTangent(Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(-58, -40, Math.toRadians(67)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(-24.34, -23.34, Math.toRadians(0)), Math.toRadians(0))
                                //.addTemporalMarker(6.9, () -> liftPosition = 1500)
                                //.addTemporalMarker(9.3, () -> cupa.setPosition(0.75))
                                //.addTemporalMarker(10.2, () -> cupa.setPosition(1))
                                //.addTemporalMarker(10.7, () -> liftPosition = 0)
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-40, -23.34), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-60, -35), Math.toRadians(270))
                                .waitSeconds(1)
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