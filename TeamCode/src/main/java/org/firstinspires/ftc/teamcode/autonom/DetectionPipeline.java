package org.firstinspires.ftc.teamcode.autonom;

import static org.opencv.core.Core.inRange;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class DetectionPipeline extends OpenCvPipeline {
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
    int capstonePosition = 0;
    Scalar scalar = new Scalar(255,120,0);
    Size size = new Size(4,4)
;
    public static class rect1{
        public static Point p1 = new Point(0,0);
        public static Point p2 = new Point(320.0/3,240);
    }

    public static class rect2{
        public static Point p1 = new Point(320.0/3,0);
        public static Point p2 = new Point(2 * 320.0/3,240);
    }

    public static class rect3{
        public static Point p1 = new Point(2 * 320.0/3,0);
        public static Point p2 = new Point(320,240);
    }

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.blur(input, blur, size);
        Imgproc.cvtColor(blur, temp, Imgproc.COLOR_BGR2HSV);
        inRange(temp, yellow_lower, yellow_upper, yellow_mask);
        input.setTo(blue, yellow_mask);

        region1_Cb = yellow_mask.submat(reg1rect);
        region2_Cb = yellow_mask.submat(reg2rect);
        region3_Cb = yellow_mask.submat(reg3rect);

        Imgproc.rectangle(input, rect1.p1, rect1.p2, scalar);
        Imgproc.rectangle(input, rect2.p1, rect2.p2, scalar);
        Imgproc.rectangle(input, rect3.p1, rect3.p2, scalar);

        int avg1,avg2,avg3;
        avg1 = (int) Core.mean(region1_Cb).val[0];
        avg2 = (int) Core.mean(region2_Cb).val[0];
        avg3 = (int) Core.mean(region3_Cb).val[0];

        int maxavg = Math.max(avg1,Math.max(avg2,avg3));

        if(maxavg == avg1){
            capstonePosition = 1;
        }

        if(maxavg == avg2){
            capstonePosition = 2;
        }

        if(maxavg == avg3){
            capstonePosition = 3;
        }

        return input;
    }

    public int getCapstonePosition(){
        return capstonePosition;
    }
}
