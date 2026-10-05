package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

@Autonomous
public class BlueSolo2Tip extends RobotOpMode {
    Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(86, 133, 90);
    private final Pose path1 = poseFactory.of(86, 117, 270);
    private final Pose point2 = poseFactory.of(132, 133, 0);
    private final Pose point2Control1 = poseFactory.of(90, 133, 0);
    private final Pose point3 = poseFactory.of(86, 31, 90);
    private final Pose point3Control1 = poseFactory.of(94, 132, 0);
    private final Pose point3Control2 = poseFactory.of(137, 24, 0);
    private final Pose point4 = poseFactory.of(94, 10, 270);
    private final Pose point5 = poseFactory.of(94, 15, 270);
    private final Pose point6 = poseFactory.of(86, 31, 90);
    private final Pose point7 = poseFactory.of(132, 26, 90);
    private Command autoRoutine() {

        return sequential(
                /*
                instant(() -> robot.flywheel.setTarget(2000)),
                instant(robot.flywheel::turnOn),
                robot.intake.on(),
                waitUntil(robot.flywheel:: atTarget),
                fire(),
                follow(follower, path1()),
                // Add mechanism commands here.
                follow(follower, path2()),

                follow(follower, path3()),
                // Add mechanism commands here.
                follow(follower, path4()),

                follow(follower, path5()),
                // Add mechanism commands here.
                follow(follower, path6())
                 */
        );
    }
    private Command fire() {
        return sequential(
                instant(robot.blocker::unblock),
                waitMs(1000),
                instant(robot.blocker::block)
        );
    }
    @Override
    public void init() {
        super.init();

        robot.blocker.block();

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {

        follower.update();
        Scheduler.execute();

    }

    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
        OpModeStorage.red = false;
        //we can add any other end state things to OpModeStorage and then also save them at this point if we need any? Probably will be useful for turret position
    }

    public Path path1() {
        return Paths.line(start, path1).constant(path1);
    }

    public Path path2() {
        return Paths.curve(path1, point2Control1, point2).constant(point2);
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3Control2, point3).linear(point2, point3);
    }

    public Path path4() {
        return Paths.line(point3, point4).constant(point4);
    }

    public Path path5() {
        return Paths.line(point4, point5).constant(point5);
    }

    public Path path6() {
        return Paths.line(point5, point6).linear(point5, point6);
    }

    public Path path7() {
        return Paths.line(point6, point7).constant(point7);
    }
}

