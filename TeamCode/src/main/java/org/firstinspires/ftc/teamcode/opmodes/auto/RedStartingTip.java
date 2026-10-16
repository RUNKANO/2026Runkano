package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.opmodes.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

@Autonomous(name = "RedStartingTip", group = "League Meet 1")
public class RedStartingTip extends RobotOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose turnPosition = poseFactory.of(56, 24, 90);
    private final Pose garden = poseFactory.of(9.5, 9.5, 180);
    private final Pose gardenControl1 = poseFactory.of(50, 9.4, 0);
    private final Pose hiveTip2Cycle1 = poseFactory.of(56, 118, 270);
    private final Pose hiveTip2Cycle1Control1 = poseFactory.of(31.1955, 18.2196, 0);
    private final Pose hiveTip2Cycle1Control2 = poseFactory.of(2, 118, 0);
    private final Pose hiveTip2Cycle1Segment1Start = poseFactory.of(56, 118, 180);
    private final Pose hiveTip2Cycle1Segment1End = poseFactory.of(56, 118, 180);
    private final Pose hiveTip2Cycle1Segment2Start = poseFactory.of(56, 118, 180);
    private final Pose hiveTip2Cycle1Segment2End = poseFactory.of(56, 118, 270);
    private final Pose flowerIntake = poseFactory.of(37, 130, 75);
    private final Pose flowerIntakeControl1 = poseFactory.of(37, 118, 0);
    private final Pose hiveTip2Cycle2 = poseFactory.of(56, 118, 270);
    private final Pose hiveTip2Cycle2Control1 = poseFactory.of(36, 118, 0);
    private final Pose park = poseFactory.of(13, 89, 270);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> robot.flywheel.setTarget(2000)),
                instant(robot.flywheel::turnOn),
                waitUntil(robot.flywheel:: atTarget),
                robot.intake.on(),
                fire(),
                robot.intake.off(),
                follow(follower, turnPosition()),
                robot.intake.on(),
                follow(follower, garden()),
                robot.intake.off(),
                follow(follower, hiveTip2Cycle1()),
                robot.intake.on(),
                fire(),
                follow(follower, flowerIntake()),
                waitMs(2000),
                follow(follower, hiveTip2Cycle2()),
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
    public void init(){
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
        OpModeStorage.red = true;
        //we can add any other end state things to OpModeStorage and then also save them at this point if we need any? Probably will be useful for turret position
    }

    public Path turnPosition() {
        return line(start, turnPosition).constant(turnPosition);
    }

    public Path garden() {
        return curve(turnPosition, gardenControl1, garden).constant(garden);
    }

    public Path hiveTip2Cycle1() {
        return curve(garden, hiveTip2Cycle1Control1, hiveTip2Cycle1Control2, hiveTip2Cycle1).heading(Interpolator.piecewise().until(0.499, Interpolator.linear(hiveTip2Cycle1Segment1Start, hiveTip2Cycle1Segment1End)).until(1, Interpolator.linear(hiveTip2Cycle1Segment2Start, hiveTip2Cycle1Segment2End)));
    }

    public Path flowerIntake() {
        return curve(hiveTip2Cycle1, flowerIntakeControl1, flowerIntake).linear(hiveTip2Cycle1, flowerIntake);
    }

    public Path hiveTip2Cycle2() {
        return curve(flowerIntake, hiveTip2Cycle2Control1, hiveTip2Cycle2).linear(flowerIntake, hiveTip2Cycle2);
    }

    public Path park() {
        return line(hiveTip2Cycle2, park).constant(park);
    }
}

