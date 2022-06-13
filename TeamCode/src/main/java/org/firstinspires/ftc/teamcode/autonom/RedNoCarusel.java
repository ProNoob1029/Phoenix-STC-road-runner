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

@Autonomous(group = "drive", name = "red no carusel")
public class RedNoCarusel extends LinearOpMode {

    DcMotorServo lift;
    Servo cupa;

    int liftPosition = 0;

    float liftSpeed = 0.4f;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(11.5, -62.4, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        lift = new DcMotorServo(hardwareMap,"lift",13.79f,28);

        DcMotorEx carusel = hardwareMap.get(DcMotorEx.class, "carusel");
        cupa = hardwareMap.get(Servo.class, "cupa");

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-8.3, -33., Math.toRadians(110)), Math.toRadians(90))
                .addTemporalMarker(2.5, () -> cupa.setPosition(0.7))
                .addTemporalMarker(3.5, () -> cupa.setPosition(1))
                .addTemporalMarker(4, () -> liftPosition = 0)
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(11.5, -61), Math.toRadians(0))
                .splineTo(new Vector2d(29.5, -61), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(37.5, -32), Math.toRadians(90))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        cupa.setPosition(1);
        liftPosition = 1400;
        drive.followTrajectorySequenceAsync(trajSeq);
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
