package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.DataLogger;

@SuppressWarnings("all")
public class Bot {
    public Gamepad gamepad1;
    public boolean orbit;
    public Drivetrain dt;

    /**
     * Constructs a new Bot, consisting of a Pedro Follower, Intake, Drivetrain, DataLogger and Gamepad
     * @param gamepad1  takes `gamepad1` or `gamepad2`
     * @param hardwareMap   takes `hardwareMap`
     */
    public Bot (Gamepad gamepad1, HardwareMap hardwareMap) {
        dt = dt.createDrivetrain(gamepad1, hardwareMap, Constants.startPose);
        dt.update();
    }

    /**
     * Runs the Drivetrain and Follower for TeleOp
     */
    public void drivetrain(Pose power, boolean orbit) {

        dt.runTeleOpDrive(
                new Pose(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x
                ),
                0.5,
                orbit,
                dt.RED_GOAL
        );

        if (gamepad1.aWasPressed()) {
            orbit = !orbit;
        }

        dt.update();
    }
}