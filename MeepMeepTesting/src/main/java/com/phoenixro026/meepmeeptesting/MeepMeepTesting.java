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
                                //.addTemporalMarker(2.3, () -> carusel.setPower(0.5))
                                //.addTemporalMarker(5.3, () -> carusel.setPower(0))
                                .waitSeconds(3)
                                .setTangent(0)
                                .splineToLinearHeading(new Pose2d(-11.67, -36, Math.toRadians(90)), Math.toRadians(90))
                                //.addTemporalMarker(6.7, () -> liftPosition = 1500)
                                //.addTemporalMarker(9.1, () -> cupa.setPosition(0.75))
                                //.addTemporalMarker(10, () -> cupa.setPosition(1))
                                //.addTemporalMarker(10.5, () -> liftPosition = 0)
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(new Vector2d(11.67, -62), Math.toRadians(0))
                                .splineTo(new Vector2d(40, -62), Math.toRadians(0))
                                .build()
                );

        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(
                        28.581263557573042,
                        28.581263557573042,
                        Math.toRadians(172.37744999999998),
                        Math.toRadians(172.37744999999998),
                        8.92
                )
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.5, -61.7, Math.toRadians(90)))
                                .setTangent(Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-11.67, -36), Math.toRadians(90))
                                //.addTemporalMarker(2.5, () -> cupa.setPosition(0.75))
                                //.addTemporalMarker(3.5, () -> cupa.setPosition(1))
                                //.addTemporalMarker(4, () -> liftPosition = 0)
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(new Vector2d(11.67, -62), Math.toRadians(0))
                                .splineTo(new Vector2d(29.5, -62), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(40, -32), Math.toRadians(90))
                                .build()
                );

        RoadRunnerBotEntity myBot3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(
                        28.581263557573042,
                        28.581263557573042,
                        Math.toRadians(172.37744999999998),
                        Math.toRadians(172.37744999999998),
                        8.92
                )
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 61.7, Math.toRadians(270)))
                                .setTangent(Math.toRadians(225))
                                .splineToLinearHeading(new Pose2d(-58, 58, Math.toRadians(45)), Math.toRadians(135))
                                //.addTemporalMarker(2.6, () -> carusel.setPower(0.5))
                                //.addTemporalMarker(5.6, () -> carusel.setPower(0))
                                .waitSeconds(3)
                                .setTangent(0)
                                .splineToLinearHeading(new Pose2d(-11.67, 36, Math.toRadians(270)), Math.toRadians(270))
                                //.addTemporalMarker(7, () -> liftPosition = 1500)
                                //.addTemporalMarker(9.4, () -> cupa.setPosition(0.75))
                                //.addTemporalMarker(10.3, () -> cupa.setPosition(1))
                                //.addTemporalMarker(10.7, () -> liftPosition = 0)
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(new Vector2d(11.67, 62), Math.toRadians(0))
                                .splineTo(new Vector2d(40, 62), Math.toRadians(0))
                                .build()
                );

        RoadRunnerBotEntity myBot4 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(
                        28.581263557573042,
                        28.581263557573042,
                        Math.toRadians(172.37744999999998),
                        Math.toRadians(172.37744999999998),
                        8.92
                )
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.5, 61.7, Math.toRadians(270)))
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-11.67, 36, Math.toRadians(270)), Math.toRadians(270))
                                //.addTemporalMarker(2.5, () -> cupa.setPosition(0.75))
                                //.addTemporalMarker(3.5, () -> cupa.setPosition(1))
                                //.addTemporalMarker(4, () -> liftPosition = 0)
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(new Vector2d(11.67, 62), Math.toRadians(0))
                                .splineTo(new Vector2d(29.5, 62), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(40, 32), Math.toRadians(270))
                                .build()
                );

        /*RoadRunnerBotEntity myBot1 = new DefaultBotBuilder(meepMeep)
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
                                .splineToLinearHeading(new Pose2d(-58, -58, Math.toRadians(135)), Math.toRadians(225))
                                //.addTemporalMarker(2.3, () -> carusel.setPower(0.5))
                                //.addTemporalMarker(5.3, () -> carusel.setPower(0))
                                .waitSeconds(3)
                                .setTangent(0)
                                .splineToLinearHeading(new Pose2d(-11.67, -36, Math.toRadians(90)), Math.toRadians(90))
                                //.addTemporalMarker(6.7, () -> liftPosition = 1500)
                                //.addTemporalMarker(9.1, () -> cupa.setPosition(0.75))
                                //.addTemporalMarker(10, () -> cupa.setPosition(1))
                                //.addTemporalMarker(10.5, () -> liftPosition = 0)
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(new Vector2d(11.67, -62), Math.toRadians(0))
                                .splineTo(new Vector2d(40, -62), Math.toRadians(0))
                                .build()
                );

        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(
                        28.581263557573042,
                        28.581263557573042,
                        Math.toRadians(172.37744999999998),
                        Math.toRadians(172.37744999999998),
                        8.92
                )
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.5, -61.7, Math.toRadians(90)))
                                .splineToConstantHeading(new Vector2d(-11.67, -36), Math.toRadians(90))
                                //.addTemporalMarker(2.5, () -> cupa.setPosition(0.75))
                                //.addTemporalMarker(3.5, () -> cupa.setPosition(1))
                                //.addTemporalMarker(4, () -> liftPosition = 0)
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(new Vector2d(11.67, -62), Math.toRadians(0))
                                .splineTo(new Vector2d(29.5, -62), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(40, -32), Math.toRadians(90))
                                .build()
                );

        RoadRunnerBotEntity myBot3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(
                        28.581263557573042,
                        28.581263557573042,
                        Math.toRadians(172.37744999999998),
                        Math.toRadians(172.37744999999998),
                        8.92
                )
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 61.7, Math.toRadians(270)))
                                .splineToLinearHeading(new Pose2d(-58, 58, Math.toRadians(45)), Math.toRadians(135))
                                //.addTemporalMarker(2.6, () -> carusel.setPower(0.5))
                                //.addTemporalMarker(5.6, () -> carusel.setPower(0))
                                .waitSeconds(3)
                                .setTangent(0)
                                .splineToLinearHeading(new Pose2d(-11.67, 36, Math.toRadians(270)), Math.toRadians(270))
                                //.addTemporalMarker(7, () -> liftPosition = 1500)
                                //.addTemporalMarker(9.4, () -> cupa.setPosition(0.75))
                                //.addTemporalMarker(10.3, () -> cupa.setPosition(1))
                                //.addTemporalMarker(10.7, () -> liftPosition = 0)
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(new Vector2d(11.67, 62), Math.toRadians(0))
                                .splineTo(new Vector2d(40, 62), Math.toRadians(0))
                                .build()
                );

        RoadRunnerBotEntity myBot4 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(
                        28.581263557573042,
                        28.581263557573042,
                        Math.toRadians(172.37744999999998),
                        Math.toRadians(172.37744999999998),
                        8.92
                )
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.5, 61.7, Math.toRadians(270)))
                                .splineToLinearHeading(new Pose2d(-11.67, 36, Math.toRadians(270)), Math.toRadians(270))
                                //.addTemporalMarker(2.5, () -> cupa.setPosition(0.75))
                                //.addTemporalMarker(3.5, () -> cupa.setPosition(1))
                                //.addTemporalMarker(4, () -> liftPosition = 0)
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(new Vector2d(11.67, 62), Math.toRadians(0))
                                .splineTo(new Vector2d(29.5, 62), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(40, 32), Math.toRadians(270))
                                .build()
                );*/

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot1)
                .addEntity(myBot2)
                .addEntity(myBot3)
                .addEntity(myBot4)
                .start();
    }
}