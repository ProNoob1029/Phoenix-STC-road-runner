package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(group = "drive", name = "motor test")
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");

        waitForStart();

        while (!isStopRequested()){
            if(gamepad1.a)
                motor.setPower(1);
            else if(gamepad1.b)
                motor.setPower(-1);
            else motor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        }
    }
}
