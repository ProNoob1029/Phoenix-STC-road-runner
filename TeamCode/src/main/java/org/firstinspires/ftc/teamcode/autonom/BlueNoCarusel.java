package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.DcMotorServo;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive", name = "blue no carusel")
public class BlueNoCarusel extends LinearOpMode {

    DcMotorServo lift;
    Servo cupa;

    int liftPosition = 0;

    float liftSpeed = 0.5f;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(11.5, 62.4, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        lift = new DcMotorServo(hardwareMap,"lift",13.79f,28);

        DcMotorEx carusel = hardwareMap.get(DcMotorEx.class, "carusel");
        cupa = hardwareMap.get(Servo.class, "cupa");

        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-11.67, 36, Math.toRadians(270)), Math.toRadians(270))
                .addTemporalMarker(2.5, () -> cupa.setPosition(0.75))
                .addTemporalMarker(3.5, () -> cupa.setPosition(1))
                .addTemporalMarker(4, () -> liftPosition = 0)
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(11.67, 61), Math.toRadians(0))
                .splineTo(new Vector2d(29.5, 61), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(37.5, 32), Math.toRadians(270))
                .build();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-11.67, 38, Math.toRadians(270)), Math.toRadians(270))
                .addTemporalMarker(2.5, () -> cupa.setPosition(0.75))
                .addTemporalMarker(3.5, () -> cupa.setPosition(1))
                .addTemporalMarker(4, () -> liftPosition = 0)
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(11.67, 61), Math.toRadians(0))
                .splineTo(new Vector2d(29.5, 61), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(37.5, 32), Math.toRadians(270))
                .build();

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-11.67, 40, Math.toRadians(270)), Math.toRadians(270))
                .addTemporalMarker(2.5, () -> cupa.setPosition(0.75))
                .addTemporalMarker(3.5, () -> cupa.setPosition(1))
                .addTemporalMarker(4, () -> liftPosition = 0)
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(11.67, 61), Math.toRadians(0))
                .splineTo(new Vector2d(29.5, 61), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(37.5, 32), Math.toRadians(270))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        cupa.setPosition(1);
        liftPosition = 450;
        drive.followTrajectorySequenceAsync(trajSeq1);
        boolean savePose = true;

        while (opModeIsActive() && !isStopRequested()){
            drive.update();
            lift.setAngle(-liftPosition, liftSpeed);
            if(!drive.isBusy() && savePose){
                savePose = false;
                PoseStorage.currentPose = drive.getPoseEstimate();
            }
        }
    }
}
