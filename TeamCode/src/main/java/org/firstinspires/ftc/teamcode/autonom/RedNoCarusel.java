package org.firstinspires.ftc.teamcode.autonom;

import static org.opencv.core.Core.inRange;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.DcMotorServo;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(group = "drive", name = "red no carusel")
public class RedNoCarusel extends LinearOpMode {

    DcMotorServo lift;
    Servo cupa;

    public static class CapstonePosition{
        public static int Position = 0;
    }
    OpenCvWebcam webcam;

    int liftPosition = 0;

    float liftSpeed = 0.5f;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(11.5, -61.7, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        lift = new DcMotorServo(hardwareMap,"lift",13.79f,28);

        //DcMotorEx carusel = hardwareMap.get(DcMotorEx.class, "carusel");
        cupa = hardwareMap.get(Servo.class, "cupa");

        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-11.67, -36, Math.toRadians(90)), Math.toRadians(90))
                .addTemporalMarker(2.5, () -> cupa.setPosition(0.75))
                .addTemporalMarker(3.5, () -> cupa.setPosition(1))
                .addTemporalMarker(4, () -> liftPosition = 0)
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(11.67, -61.5), Math.toRadians(0))
                .splineTo(new Vector2d(29.5, -61.5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40, -32), Math.toRadians(90))
                .build();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-11.67, -37, Math.toRadians(90)), Math.toRadians(90))
                .addTemporalMarker(2.5, () -> cupa.setPosition(0.75))
                .addTemporalMarker(3.5, () -> cupa.setPosition(1))
                .addTemporalMarker(4, () -> liftPosition = 0)
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(11.67, -61.5), Math.toRadians(0))
                .splineTo(new Vector2d(29.5, -61.5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40, -32), Math.toRadians(90))
                .build();

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-11.67, -38, Math.toRadians(90)), Math.toRadians(90))
                .addTemporalMarker(2.5, () -> cupa.setPosition(0.75))
                .addTemporalMarker(3.5, () -> cupa.setPosition(1))
                .addTemporalMarker(4, () -> liftPosition = 0)
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(11.67, -61.5), Math.toRadians(0))
                .splineTo(new Vector2d(29.5, -61.5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40, -32), Math.toRadians(90))
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new SamplePipeline());


        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis", CapstonePosition.Position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        int position = CapstonePosition.Position;
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

    static class SamplePipeline extends OpenCvPipeline
    {
        Mat blur = new Mat();
        Mat yellow_mask = new Mat();
        Mat temp= new Mat();
        Mat region1_Cb, region2_Cb, region3_Cb;
        Scalar yellow_lower = new Scalar(80, 100, 100);
        Scalar yellow_upper = new Scalar(100, 255, 255);
        Scalar blue = new Scalar(0, 0, 255);
        Rect reg1rect = new Rect(new Point(0,0), new Point(320.0/3,240));
        Rect reg2rect = new Rect(new Point(320.0/3,0), new Point(2 * 320.0/3,240));
        Rect reg3rect = new Rect(new Point(2 * 320.0/3,0), new Point(320,240));

        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.blur(input,blur,new Size(4,4));
            Imgproc.cvtColor(blur,temp,Imgproc.COLOR_BGR2HSV);
            inRange(temp, yellow_lower, yellow_upper, yellow_mask);
            input.setTo(blue, yellow_mask);

            region1_Cb = yellow_mask.submat(reg1rect);
            region2_Cb = yellow_mask.submat(reg2rect);
            region3_Cb = yellow_mask.submat(reg3rect);

            Imgproc.rectangle(input,new Point(0,0), new Point(320.0/3,240),new Scalar(255,120,0));
            Imgproc.rectangle(input,new Point(320.0/3,0), new Point(2 * 320.0/3,240),new Scalar(255,120,0));
            Imgproc.rectangle(input,new Point(2 * 320.0/3,0), new Point(320,240),new Scalar(255,120,0));

            int avg1,avg2,avg3;
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            int maxavg = Math.max(avg1,Math.max(avg2,avg3));

            if(maxavg == avg1){
                CapstonePosition.Position = 1;
            }

            if(maxavg == avg2){
                CapstonePosition.Position = 2;
            }

            if(maxavg == avg3){
                CapstonePosition.Position = 3;
            }

            return input;
        }
    }
}
