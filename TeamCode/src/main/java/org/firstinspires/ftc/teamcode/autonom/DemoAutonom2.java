package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.DcMotorServo;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive", name = "STC Auto 2")
public class DemoAutonom2 extends OpMode {

    DcMotorServo lift;

    SampleMecanumDrive drive;

    int liftPosition = 0;

    float liftSpeed = 0.2f;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(11.5, -62.4, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        lift = new DcMotorServo(hardwareMap,"lift",13.79f,28);

        DcMotorEx carusel = hardwareMap.get(DcMotorEx.class, "carusel");

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-6.5, -38.5), Math.toRadians(110))
                .addTemporalMarker(0.5, () -> {
                    liftPosition = 1350;
                    carusel.setPower(0.5);
                })
                .build();

        drive.followTrajectorySequenceAsync(trajSeq);
    }

    @Override
    public void loop() {
        drive.update();
        lift.setAngle(-liftPosition, liftSpeed);
    }
}
