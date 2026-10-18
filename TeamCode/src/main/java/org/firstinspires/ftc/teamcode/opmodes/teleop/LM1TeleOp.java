package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
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
    private boolean fCentric = false;

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

        //fix this for whichever side the red and blue are on for field centric before using and the last one is robot centric for if field centric gets off
        if(OpModeStorage.red && fCentric){
            DrivePowers powers = ManualDrive.fieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, follower.pose().heading());
            ManualDrive.driveOrHold(follower, powers);
            follower.manual(powers);
        }else if(fCentric){
            DrivePowers powers = ManualDrive.fieldCentric(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x, follower.pose().heading());
            ManualDrive.driveOrHold(follower, powers);
            follower.manual(powers);
        }else{
            follower.manual(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        if(gamepad1.rightTriggerWasPressed()) robot.intake.on().schedule();
        if(gamepad1.rightTriggerWasReleased()) robot.intake.off().schedule();

        if(gamepad1.left_trigger > .1 && robot.flywheel.atTarget()){
                robot.intake.fire().schedule();
        }else if(gamepad1.left_trigger >.1){
            robot.intake.off().schedule();
        }
        if(gamepad1.leftTriggerWasPressed()) robot.blocker.unblock();
        if (gamepad1.leftTriggerWasReleased()) {
            robot.blocker.block();
            if(!(gamepad1.right_trigger > .1)) robot.intake.off().schedule();
        }

        if(gamepad1.dpadUpWasPressed()) robot.flywheel.setTarget(robot.flywheel.target +=20);
        if(gamepad1.dpadDownWasPressed()) robot.flywheel.setTarget(robot.flywheel.target -=20);

        if(gamepad1.bWasPressed()) robot.flywheel.toggle();

        if(gamepad1.backWasPressed()) fCentric = !fCentric;

        if(gamepad1.startWasPressed() && OpModeStorage.red){
                Pose redGardenPose = new Pose(8.5, 8.5, Math.toRadians(270));
                follower.setPose(redGardenPose);
        }else if(gamepad1.startWasPressed()){
            Pose blueGardenPose = new Pose(133.5, 133.5, Math.toRadians(90));
            follower.setPose(blueGardenPose);
        }

        //in case of alliance being saved incorrectly or for testing
        if(gamepad2.aWasPressed()) OpModeStorage.red = !OpModeStorage.red;

        follower.update();
        Scheduler.execute();

        Pose robotPose = follower.pose();
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        telemetry.addData("Flywheel Velocity Target", robot.flywheel.target);
    }
}
