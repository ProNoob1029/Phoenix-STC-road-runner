package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "drive", name = "non field centric")
@Config
public class NonFieldCentric extends SampleTeleOp {

    @Override
    public Vector2d getDriveInput(double leftX, double leftY, double heading) {
        return new Vector2d(
                leftY,
                -leftX
        );
    }

    @Override
    public void setCaruselPower() {
        caruselLowPower = 0.5;
        caruselHighPower = 0.7;
    }
}
