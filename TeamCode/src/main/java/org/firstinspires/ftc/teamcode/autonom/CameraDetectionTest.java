package org.firstinspires.ftc.teamcode.autonom;

import static org.opencv.core.Core.inRange;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

@Autonomous(group = "drive", name = "camera detection test")
public class CameraDetectionTest extends LinearOpMode {

    public static class CapstonePosition{
        public static int Position;
    }
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {
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

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            telemetry.addData("pos", CapstonePosition.Position);
            telemetry.update();
        }
    }

    static class SamplePipeline extends OpenCvPipeline
    {
        Mat blur = new Mat();
        Mat yellow_mask = new Mat();
        Mat temp= new Mat();
        Mat region1_Cb, region2_Cb, region3_Cb;
        Scalar yellow_lower = new Scalar(0, 100, 100);
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
