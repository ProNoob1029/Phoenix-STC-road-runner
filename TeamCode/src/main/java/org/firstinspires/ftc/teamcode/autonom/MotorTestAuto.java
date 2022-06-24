package org.firstinspires.ftc.teamcode.autonom;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(group = "drive", name = "motor test auto")
public class MotorTestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");

        waitForStart();

        while (opModeIsActive()){
            motor.setPower(1);
        }
    }
}
