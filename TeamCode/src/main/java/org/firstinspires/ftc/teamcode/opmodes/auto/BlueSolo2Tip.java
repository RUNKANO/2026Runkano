package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.interpolator.Interpolator;
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

@Autonomous(name = "BlueStartingTip", group = "League Meet 1")
public class BlueSolo2Tip extends RobotOpMode {
    Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(86, 133, 90);
    private final Pose turnPos = poseFactory.of(86, 117, 180);
    private final Pose garden = poseFactory.of(132, 133, -179.8904);
    private final Pose gardenControl1 = poseFactory.of(90, 133, 0);
    private final Pose tip2Cycle1 = poseFactory.of(90, 18, 90);
    private final Pose tip2Cycle1Control1 = poseFactory.of(94, 133, 0);
    private final Pose tip2Cycle1Control2 = poseFactory.of(132, 18, 0);
    private final Pose flowerIntake = poseFactory.of(85, 11.5, 285);
    private final Pose tip2Cycle2 = poseFactory.of(85, 18, 450);
    private final Pose tip2Cycle2Segment1Start = poseFactory.of(85, 18, 285);
    private final Pose tip2Cycle2Segment1End = poseFactory.of(85, 18, 285);
    private final Pose tip2Cycle2Segment2Start = poseFactory.of(85, 18, 285);
    private final Pose tip2Cycle2Segment2End = poseFactory.of(85, 18, 90);
    private final Pose park = poseFactory.of(130, 50, 90);
    private Command autoRoutine() {

        return sequential(
                instant(() -> robot.flywheel.setTarget(2000)),
                instant(robot.flywheel::turnOn),
                robot.intake.on(),
                waitUntil(robot.flywheel:: atTarget),
                fire(),
                robot.intake.off(),
                follow(follower, turnPos()),
                robot.intake.on(),
                follow(follower, garden()),
                robot.intake.off(),
                follow(follower, tip2Cycle1()),
                instant(()-> robot.flywheel.setTarget(1700)),
                waitUntil(robot.flywheel::atTarget),
                robot.intake.on(),
                fire(),
                follow(follower, flowerIntake()),
                waitMs(2000),
                robot.intake.off(),
                follow(follower, tip2Cycle2()),
                waitUntil(robot.flywheel::atTarget),
                robot.intake.on(),
                fire(),
                robot.intake.off(),
                follow(follower, park())
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
        OpModeStorage.autonomousEndPose = follower.pose();
        OpModeStorage.red = false;
        //we can add any other end state things to OpModeStorage and then also save them at this point if we need any? Probably will be useful for turret position
    }

    public Path turnPos() {
        return line(start, turnPos).linear(start, turnPos);
    }

    public Path garden() {
        return curve(turnPos, gardenControl1, garden).reverseTangent();
    }

    public Path tip2Cycle1() {
        return curve(garden, tip2Cycle1Control1, tip2Cycle1Control2, tip2Cycle1).constant(tip2Cycle1);
    }

    public Path flowerIntake() {
        return line(tip2Cycle1, flowerIntake).constant(flowerIntake);
    }

    public Path tip2Cycle2() {
        return line(flowerIntake, tip2Cycle2).heading(Interpolator.piecewise().until(0.7505, Interpolator.linear(tip2Cycle2Segment1Start, tip2Cycle2Segment1End)).until(1, Interpolator.linear(tip2Cycle2Segment2Start, tip2Cycle2Segment2End)));
    }

    public Path park() {
        return line(tip2Cycle2, park).constant(park);
    }
}

