package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "drive", name = "red field centric")
@Config
public class RedFieldCentric extends SampleTeleOp {

    @Override
    public Vector2d getDriveInput(double leftX, double leftY, double heading) {
        return new Vector2d(
                leftX,
                leftY
        ).rotated(-heading);
    }

    @Override
    public void setCaruselPower() {
        caruselLowPower = 0.5;
        caruselHighPower = 0.7;
    }
}
