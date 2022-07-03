package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive", name = "blue with carusel")
public class BlueWithCarusel extends SampleAutonom {

    int liftPosition = 0;
    float liftSpeed = 0.5f;

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();

        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(-58, 58, Math.toRadians(45)), Math.toRadians(135))
                .addTemporalMarker(2.6, () -> carusel.setPower(-0.4))
                .addTemporalMarker(5.6, () -> carusel.setPower(0))
                .waitSeconds(3)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-11.67, 36, Math.toRadians(270)), Math.toRadians(270))
                .addTemporalMarker(7, () -> liftPosition = 1500)
                .addTemporalMarker(10.4, () -> cupa.setPosition(0.75))
                .addTemporalMarker(11.3, () -> cupa.setPosition(1))
                .addTemporalMarker(11.7, () -> liftPosition = 0)
                .waitSeconds(2)
                .setReversed(true)
                .splineTo(new Vector2d(11.67, 61.5), Math.toRadians(0))
                .splineTo(new Vector2d(42, 61.5), Math.toRadians(0))
                .waitSeconds(1)
                .build();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(-58, 58, Math.toRadians(45)), Math.toRadians(135))
                .addTemporalMarker(2.6, () -> carusel.setPower(-0.4))
                .addTemporalMarker(5.6, () -> carusel.setPower(0))
                .waitSeconds(3)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-11.67, 37, Math.toRadians(270)), Math.toRadians(270))
                .addTemporalMarker(7, () -> liftPosition = 950)
                .addTemporalMarker(10.4, () -> cupa.setPosition(0.75))
                .addTemporalMarker(11.3, () -> cupa.setPosition(1))
                .addTemporalMarker(11.7, () -> liftPosition = 0)
                .waitSeconds(2)
                .setReversed(true)
                .splineTo(new Vector2d(11.67, 61.5), Math.toRadians(0))
                .splineTo(new Vector2d(42, 61.5), Math.toRadians(0))
                .waitSeconds(1)
                .build();

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(-58, 58, Math.toRadians(45)), Math.toRadians(135))
                .addTemporalMarker(2.6, () -> carusel.setPower(-0.4))
                .addTemporalMarker(5.6, () -> carusel.setPower(0))
                .waitSeconds(3)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-11.67, 38, Math.toRadians(270)), Math.toRadians(270))
                .addTemporalMarker(7, () -> liftPosition = 450)
                .addTemporalMarker(10.4, () -> cupa.setPosition(0.75))
                .addTemporalMarker(11.3, () -> cupa.setPosition(1))
                .addTemporalMarker(11.7, () -> liftPosition = 0)
                .waitSeconds(2)
                .setReversed(true)
                .splineTo(new Vector2d(11.67, 61.5), Math.toRadians(0))
                .splineTo(new Vector2d(42, 61.5), Math.toRadians(0))
                .waitSeconds(1)
                .build();

        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis", pipeline.getCapstonePosition());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        int position = pipeline.getCapstonePosition();
        //webcam.closeCameraDeviceAsync(() -> webcam.stopStreaming());

        if (isStopRequested()) return;

        cupa.setPosition(1);
        liftPosition = 0;

        switch (position){
            case 1:
                drive.followTrajectorySequenceAsync(trajSeq1);
                break;
            case 2:
                drive.followTrajectorySequenceAsync(trajSeq2);
                break;
            case 3:
                drive.followTrajectorySequenceAsync(trajSeq3);
                break;
        }

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
