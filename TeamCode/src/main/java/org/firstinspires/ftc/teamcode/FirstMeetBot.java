package org.firstinspires.ftc.teamcode;


import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.DataLogger;

import java.lang.Exception;
import java.util.function.Supplier;

@SuppressWarnings("all")
public class FirstMeetBot {
    //public Gamepad gamepad1;
    public boolean orbit;
    public Drivetrain dt;
    public Path shootPath;
    public boolean goTo;
    public Supplier<PathChain> pathChain;

    /**
     * Constructs a new Bot, consisting of a Pedro Follower, Intake,
     Drivetrain, DataLogger and Gamepad
     * @param gamepad1  takes `gamepad1` or `gamepad2`
     * @param hardwareMap   takes `hardwareMap`
     */
    public FirstMeetBot(Gamepad gamepad, HardwareMap hwMap) throws Exception{
        try {
            dt = new Drivetrain(gamepad, hwMap, Constants.startPose);
            dt.update();
        }
        catch (RuntimeException ignored) {
            throw new RobotCoreException("Failed to Initialize Drivetrain");
        }

        if (dt == null) {   // Ends program and throws error to RC Phone if `dt` is not initialized properly
            throw new RobotCoreException("Drivetrain returned `null`, line 30", new Exception());
        }

        orbit = false;
        goTo = false;
        shootPath = new Path();

        pathChain = () -> dt.follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(dt.follower::getPose, dt.RED_PICK_POS)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(dt.follower::getHeading, dt.RED_PICK_POS.getHeading(), 0.8))
                .build();
    }

    /**
     * Runs the Drivetrain and Follower for TeleOp
     */
    public void drivetrain(Gamepad gamepad) {
        dt.update();

        if (!goTo) {
            dt.runTeleOpDrive(
                    gamepad.left_stick_y,     //  Forward
                    gamepad.left_stick_x,      //  Strafe
                    gamepad.right_stick_x,      //  Rotation
                    0.05,                        //  Drive Power Relative to Input
                    0.9,
                    orbit,                      //  Boolean: Should the robot orbit around the goal?
                    dt.RED_GOAL,                 //  Pose object indicating where to orbit around
                    gamepad
            );
        }
        else if (goTo) {
            toShootPos(shootPath);
        }

        if (gamepad.aWasPressed()) {       //Toggle goTo

            if (goTo) {
                dt.follower.breakFollowing();
                dt.follower.startTeleopDrive(true);
            }
            else {
                dt.follower.followPath(pathChain.get());
            }
            goTo = !goTo;
        }

    }

    private void toShootPos(Path shootPath) {
        dt.follower.followPath(shootPath);
    }
}