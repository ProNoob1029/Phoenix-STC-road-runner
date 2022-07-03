package org.firstinspires.ftc.teamcode.autonom;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "drive", name = "camera detection test")
public class CameraDetectionTest extends SampleAutonom {

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            telemetry.addData("pos", pipeline.getCapstonePosition());
            telemetry.update();
        }
    }
}
