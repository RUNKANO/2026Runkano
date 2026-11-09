package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

@Autonomous(name = "RedFar1Tip", group = "Autonomous")
public class RedFar1Tip extends RobotOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(32, 9, 270);
    private final Pose cycle1 = poseFactory.of(32, 24, 270);
    private final Pose garden = poseFactory.of(10, 10, 270);
    private final Pose cycle2 = poseFactory.of(32, 24, 270);
    private final Pose park = poseFactory.of(16, 94, 270);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> robot.flywheel.setTarget(2200)),
                instant(robot.flywheel::turnOn),
                follow(follower, cycle1()),
                instant(()->robot.turret.turretToPos(((double)140/250))),
                waitMs(300),
                waitUntil(robot.flywheel:: atTarget),
                fire(),
                follow(follower, garden()),
                follow(follower, cycle2()),
                fire(),
                instant(()->robot.intake.off()),
                instant(robot.flywheel::turnOff),
                follow(follower, park())
        );
    }
    private Command fire() {
        return sequential(
                instant(robot.blocker::unblock),
                waitMs(50),
                robot.intake.fire(),
                waitMs(450),
                instant(robot.blocker::block),
                robot.intake.on()
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
        telemetry.update();
    }
    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose();
        OpModeStorage.red = true;
        //we can add any other end state things to OpModeStorage and then also save them at this point if we need any? Probably will be useful for turret position
    }

    public Path cycle1() {
        return line(start, cycle1).constant(cycle1);
    }

    public Path garden() {
        return line(cycle1, garden).constant(garden);
    }

    public Path cycle2() {
        return line(garden, cycle2).constant(cycle2);
    }

    public Path park() {
        return line(cycle2, park).constant(park);
    }
}

