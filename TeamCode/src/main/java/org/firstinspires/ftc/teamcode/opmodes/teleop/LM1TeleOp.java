package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;

@TeleOp(name = "LM1 Teleop", group = "League Meet 1")
@Config
public class LM1TeleOp extends RobotOpMode {
    private Follower follower;
    private boolean fCentric = true;

    @Override
    public void init(){
        super.init();

        follower = Constants.create(hardwareMap);
        robot.blocker.block();
        follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();
    }

    @Override
    public void start(){
        robot.flywheel.turnOn();
    }

    @Override
    public void loop(){
        /*
        fix this for whichever side the red and blue are on for field centric before using and the last one is robot centric for if field centric gets off
        if(OpModeStorage.red && fCentric){
            DrivePowers powers = ManualDrive.fieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, follower.pose().heading());
        }else if(fCentric){
            DrivePowers powers = ManualDrive.fieldCentric(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x, follower.pose().heading());
        }else{
                    follower.manual(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
         */
        DrivePowers powers = ManualDrive.fieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, follower.pose().heading());
        if(fCentric) {
            //follower.manual(powers); <-- this will not hold the position of the robot after the sticks are let go, if we have problems or another driver has a preference remove the comment, I could bind it to a button but guh maybe I'll add it to controller 2 or something
            ManualDrive.driveOrHold(follower, powers);
        }
        follower.update();
        Pose robotPose = follower.pose();

        if(gamepad1.rightTriggerWasPressed()) robot.intake.on().schedule();
        if(gamepad1.rightTriggerWasReleased()) robot.intake.off().schedule();

        if(gamepad1.leftBumperWasPressed()){
            robot.blocker.unblock();
            robot.intake.on().schedule();
        }
        if (gamepad1.leftBumperWasReleased()) {
            robot.blocker.block();
            robot.intake.off().schedule();
        }

        if(gamepad1.bWasPressed()) robot.flywheel.toggle();

        if(gamepad1.backWasPressed()) fCentric = !fCentric;

        if(gamepad1.startWasPressed()){
            if(!OpModeStorage.red){
                Pose blueCornerPose = new Pose(10.5, 10.5, Math.toRadians(0));
                follower.setPose(blueCornerPose);
            }else{
                Pose redCornerPose = new Pose(133.5, 133.5, Math.toRadians(180)); //have to find what the actual position is for the gardens or something
                follower.setPose(redCornerPose);
            }
        }

        //in case of alliance being saved incorrectly or for testing
        if(gamepad2.aWasPressed()) OpModeStorage.red = !OpModeStorage.red;

        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
    }
}
