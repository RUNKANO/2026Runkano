package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

@Autonomous(name = "Blue1Cycle", group = "Autonomous")
public class Blue1Cycle extends RobotOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(109, 133, 90);
    private final Pose cycle1 = poseFactory.of(109, 118, 90);
    private final Pose park = poseFactory.of(125, 49, 90);

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
                waitMs(600),
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


    public Path park() {
        return line(cycle1, park).constant(park);
    }
}

