package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import java.util.List;


@SuppressWarnings("all")
@TeleOp (name="TeleKorver")
public class TeleKorver extends OpMode {
    private FirstMeetBot bot;
    private Pose drivePower;

    @Override
    public void init () {
        try {
            bot = new FirstMeetBot(gamepad1, hardwareMap);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        drivePower = new Pose();
    }

    @Override
    public void start() {
        bot.dt.setBrakeMode(true);
    }

    @Override
    public void loop () {
        bot.drivetrain(gamepad1);
        bot.dt.update();
        telemetry.addData("Bot X:", bot.dt.follower.getPose().getX());
        telemetry.addData("Bot Y:", bot.dt.follower.getPose().getY());
        telemetry.addData("Bot Theta: ", bot.dt.follower.getPose().getHeading());
        telemetry.update();
    }
}
