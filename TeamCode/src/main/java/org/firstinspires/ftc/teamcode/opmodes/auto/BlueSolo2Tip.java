package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

@Autonomous(name = "Blue solo 2 tip", group = "League Meet 2")
public class BlueSolo2Tip extends RobotOpMode {
    Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();


    private final Pose start = poseFactory.of(108.75, 133.75, 90);
    private final Pose cycle1 = poseFactory.of(118.5, 118, 90);
    private final Pose garden = poseFactory.of(131.75, 131.75, 90);
    private final Pose t2Cycle1 = poseFactory.of(94.75, 22, -90);
    private final Pose t2Cycle1Control1 = poseFactory.of(103.75, 131.75, 0);
    private final Pose t2Cycle1Control2 = poseFactory.of(120.75, 24, 0);
    private final Pose t2Cycle1Segment1Heading = poseFactory.of(94.75, 22, 90);
    private final Pose t2Cycle1Segment2Start = poseFactory.of(94.75, 22, 90);
    private final Pose t2Cycle1Segment2End = poseFactory.of(94.75, 22, 270);
    private final Pose t2Cycle1Segment3Heading = poseFactory.of(94.75, 22, -90);
    private final Pose flowerIntakeCycle2 = poseFactory.of(94.75, 14, 270);
    private final Pose flowerIntake2 = poseFactory.of(129.75, 95, 0);
    private final Pose flowerIntake2Control1 = poseFactory.of(117.75, 20, 0);
    private final Pose flowerIntake2Control2 = poseFactory.of(101.75, 95, 0);
    private final Pose flowerIntake2Segment1Heading = poseFactory.of(129.75, 95, 270);
    private final Pose flowerIntake2Segment2Start = poseFactory.of(129.75, 95, -90);
    private final Pose flowerIntake2Segment2End = poseFactory.of(129.75, 95, 0);
    private final Pose flowerIntake2Segment3Heading = poseFactory.of(129.75, 95, 0);
    private final Pose insuranceCycle = poseFactory.of(106.75, 25, 0);
    private final Pose insuranceCycleControl1 = poseFactory.of(108.7411, 79, 0);
    private final Pose park = poseFactory.of(124.75, 54.75, 0);
    private final Pose parkControl1 = poseFactory.of(105.75, 54.75, 0);
    private Command autoRoutine() {

        return sequential(
                instant(() -> robot.flywheel.setTarget(2240)),
                instant(robot.flywheel::turnOn),
                instant(()->robot.turret.turretToPos((double)110/250)),
                waitMs(300),
                follow(follower, cycle1()),
                fire(),
                follow(follower, garden()),
                instant(()->robot.turret.turretToPos((double)80/250)),
                follow(follower, t2Cycle1()),
                fire(),
                instant(()->robot.flowerIntake.down()),
                follow(follower, flowerIntakeCycle2()),
                fire(),
                follow(follower, flowerIntake2()),
                follow(follower, insuranceCycle()),
                instant(()->robot.turret.turretToPos(1-(double)130/250)),
                fire(),
                follow(follower, park())
        );
    }
    private Command fire() {
        return sequential(
                robot.intake.fire(),
                instant(robot.blocker::unblock),
                waitMs(500),
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

    }

    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose();
        OpModeStorage.red = false;
        //we can add any other end state things to OpModeStorage and then also save them at this point if we need any? Probably will be useful for turret position
    }

    public Path cycle1() {
        return Paths.line(start, cycle1).constant(cycle1);
    }

    public Path garden() {
        return Paths.line(cycle1, garden).constant(garden);
    }

    public Path t2Cycle1() {
        return Paths.curve(garden, t2Cycle1Control1, t2Cycle1Control2, t2Cycle1).heading(Interpolator.piecewise().until(0.25, Interpolator.constant(t2Cycle1Segment1Heading)).until(0.75, Interpolator.linear(t2Cycle1Segment2Start, t2Cycle1Segment2End)).until(1, Interpolator.constant(t2Cycle1Segment3Heading)));
    }

    public Path flowerIntakeCycle2() {
        return Paths.line(t2Cycle1, flowerIntakeCycle2).constant(flowerIntakeCycle2);
    }

    public Path flowerIntake2() {
        return Paths.curve(flowerIntakeCycle2, flowerIntake2Control1, flowerIntake2Control2, flowerIntake2).heading(Interpolator.piecewise().until(0.25, Interpolator.constant(flowerIntake2Segment1Heading)).until(0.75, Interpolator.linear(flowerIntake2Segment2Start, flowerIntake2Segment2End)).until(1, Interpolator.constant(flowerIntake2Segment3Heading)));
    }

    public Path insuranceCycle() {
        return Paths.curve(flowerIntake2, insuranceCycleControl1, insuranceCycle).constant(insuranceCycle);
    }

    public Path park() {
        return Paths.curve(insuranceCycle, parkControl1, park).constant(park);
    }
}

