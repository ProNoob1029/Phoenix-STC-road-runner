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

@Autonomous(group = "drive", name = "blue with carusel")
public class BlueWithCarusel extends LinearOpMode {

    DcMotorServo lift;
    Servo cupa;

    int liftPosition = 0;

    float liftSpeed = 0.5f;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35, 62.4, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        lift = new DcMotorServo(hardwareMap,"lift",13.79f,28);

        DcMotorEx carusel = hardwareMap.get(DcMotorEx.class, "carusel");
        cupa = hardwareMap.get(Servo.class, "cupa");

        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-59, 59, Math.toRadians(45)), Math.toRadians(135))
                .addTemporalMarker(2.6, () -> carusel.setPower(0.5))
                .addTemporalMarker(5.6, () -> carusel.setPower(0))
                .waitSeconds(3)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-11.67, 36, Math.toRadians(270)), Math.toRadians(270))
                .addTemporalMarker(7, () -> liftPosition = 1500)
                .addTemporalMarker(9.4, () -> cupa.setPosition(0.75))
                .addTemporalMarker(10.3, () -> cupa.setPosition(1))
                .addTemporalMarker(10.7, () -> liftPosition = 0)
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(11.67, 60), Math.toRadians(0))
                .splineTo(new Vector2d(37.5, 60), Math.toRadians(0))
                .build();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-59, 59, Math.toRadians(45)), Math.toRadians(135))
                .addTemporalMarker(2.6, () -> carusel.setPower(0.5))
                .addTemporalMarker(5.6, () -> carusel.setPower(0))
                .waitSeconds(3)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-11.67, 38, Math.toRadians(270)), Math.toRadians(270))
                .addTemporalMarker(7, () -> liftPosition = 950)
                .addTemporalMarker(9.4, () -> cupa.setPosition(0.75))
                .addTemporalMarker(10.3, () -> cupa.setPosition(1))
                .addTemporalMarker(10.7, () -> liftPosition = 0)
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(11.67, 60), Math.toRadians(0))
                .splineTo(new Vector2d(37.5, 60), Math.toRadians(0))
                .build();

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-59, 59, Math.toRadians(45)), Math.toRadians(135))
                .addTemporalMarker(2.6, () -> carusel.setPower(0.5))
                .addTemporalMarker(5.6, () -> carusel.setPower(0))
                .waitSeconds(3)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-11.67, 40, Math.toRadians(270)), Math.toRadians(270))
                .addTemporalMarker(7, () -> liftPosition = 450)
                .addTemporalMarker(9.4, () -> cupa.setPosition(0.75))
                .addTemporalMarker(10.3, () -> cupa.setPosition(1))
                .addTemporalMarker(10.7, () -> liftPosition = 0)
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(11.67, 60), Math.toRadians(0))
                .splineTo(new Vector2d(37.5, 60), Math.toRadians(0))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        cupa.setPosition(1);
        liftPosition = 0;
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
