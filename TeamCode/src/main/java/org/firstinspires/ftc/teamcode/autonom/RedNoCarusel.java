package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive", name = "red no carusel")
public class RedNoCarusel extends SampleAutonom {

    int liftPosition = 0;
    float liftSpeed = 0.5f;

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();

        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-11.67, -35.6, Math.toRadians(90)), Math.toRadians(90))
                .addTemporalMarker(3.5, () -> cupa.setPosition(0.75))
                .addTemporalMarker(4.5, () -> cupa.setPosition(1))
                .addTemporalMarker(5, () -> liftPosition = 0)
                .waitSeconds(2)
                .setReversed(true)
                .splineTo(new Vector2d(11.67, -61.5), Math.toRadians(0))
                .splineTo(new Vector2d(29.5, -61.5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(41, -32), Math.toRadians(90))
                .build();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-11.67, -37, Math.toRadians(90)), Math.toRadians(90))
                .addTemporalMarker(3.5, () -> cupa.setPosition(0.75))
                .addTemporalMarker(4.5, () -> cupa.setPosition(1))
                .addTemporalMarker(5, () -> liftPosition = 0)
                .waitSeconds(2)
                .setReversed(true)
                .splineTo(new Vector2d(11.67, -61.5), Math.toRadians(0))
                .splineTo(new Vector2d(29.5, -61.5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(41, -32), Math.toRadians(90))
                .build();

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-11.67, -38, Math.toRadians(90)), Math.toRadians(90))
                .addTemporalMarker(3.5, () -> cupa.setPosition(0.75))
                .addTemporalMarker(4.5, () -> cupa.setPosition(1))
                .addTemporalMarker(5, () -> liftPosition = 0)
                .waitSeconds(2)
                .setReversed(true)
                .splineTo(new Vector2d(11.67, -61.5), Math.toRadians(0))
                .splineTo(new Vector2d(29.5, -61.5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(41, -32), Math.toRadians(90))
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
        switch (position){
            case 1:
                drive.followTrajectorySequenceAsync(trajSeq1);
                liftPosition = 450;
                break;
            case 2:
                drive.followTrajectorySequenceAsync(trajSeq2);
                liftPosition = 950;
                break;
            case 3:
                drive.followTrajectorySequenceAsync(trajSeq3);
                liftPosition = 1500;
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
