package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.OpModeStorage;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Config
public class Turret {
    private final Servo turretServo1;
    private final Servo turretServo2;
    private final Telemetry telemetry;
    private static final double MM_TO_IN = 1.0 / 25.4;
    // turret's offset from the robot's center along the robot's FORWARD axis (inches, +forward)
    public static double TURRET_OFFSET_FORWARD_IN = -25 * MM_TO_IN;
    // turret's offset from the robot's center along the robot's LATERAL axis (inches, +left)
    public static double TURRET_OFFSET_LATERAL_IN = 0 * MM_TO_IN;
    public double turretAngleCWDeg;
    public static double TURRET_FORWARD_SERVO_POS = 0;
    public static double TURRET_TOTAL_TRAVEL_DEG = 250;
    public static boolean TURRET_CW_POSITIVE = false;
    public static Pose TRACK_TARGET_DEFAULT = new Pose(135, 142, 0);
    public static Pose RED_HIVE_GARDEN = new Pose(57, 52, 0);
    public static Pose RED_HIVE_FLOWER = new Pose(58, 88, 0);
    public static Pose BLUE_HIVE_GARDEN = new Pose(84, 52, 0);
    public static Pose BLUE_HIVE_FLOWER = new Pose(84, 88, 0);
    public static Pose RELOCALIZATION_RED_GARDEN = new Pose(9, 9, 270);
    public static Pose RELOCALIZATION_BLUE_GARDEN = new Pose(135, 135, 90);
    public double trackTargetX;
    public double trackTargetY;
    Pose trackTarget = TRACK_TARGET_DEFAULT;
    public Follower follower;
    public boolean targetInDeadZone = false;
    private boolean tracking = false;
    private boolean hiveAudienceSide = false;

    public Turret(Robot robot) {
        turretServo1 = robot.hardwareMap.get(Servo.class, "turret1");
        turretServo2 = robot.hardwareMap.get(Servo.class, "turret2");
        telemetry = robot.telemetry;

        turretAngleCWDeg = 0; //start pointed forward
        turretToPos(TURRET_FORWARD_SERVO_POS);
    }

    public void turretToPos(double position){
        turretServo1.setPosition(1-position);
        turretServo2.setPosition(1-position);
    }
    ///
    /// Angle Servo Conversions
    ///
    public double angleCWDegToServoPos(double angleCWDeg) {
        double normalized = angleCWDeg / TURRET_TOTAL_TRAVEL_DEG; // 0 at forward, 1 at full travel
        return TURRET_CW_POSITIVE
                ? TURRET_FORWARD_SERVO_POS - normalized
                : TURRET_FORWARD_SERVO_POS + normalized;
    }

    public double servoPosToAngleCWDeg(double servoPos) {
        double normalized = TURRET_CW_POSITIVE
                ? TURRET_FORWARD_SERVO_POS - servoPos
                : servoPos - TURRET_FORWARD_SERVO_POS;
        return normalized * TURRET_TOTAL_TRAVEL_DEG;
    }
    ///
    /// Wrap Around Fixes
    ///
    // wraps radians into (-pi, pi]
    private double normalizeRadians(double rad) {
        double twoPi = 2 * Math.PI;
        return ((((rad + Math.PI) % twoPi) + twoPi) % twoPi) - Math.PI;
    }

    // wraps degrees into [0, 360)
    private double normalizeDegrees0to360(double deg) {
        double wrapped = deg % 360.0;
        return wrapped < 0 ? wrapped + 360.0 : wrapped;
    }

    public double resolveWraparound(double rawAngleCWDeg, double currentAngleCWDeg) {
        double best = Double.NaN;
        double bestDist = Double.POSITIVE_INFINITY;

        for (int k = -1; k <= 1; k++) {
            double candidate = rawAngleCWDeg + 360.0 * k;
            if (candidate >= 0 && candidate <= TURRET_TOTAL_TRAVEL_DEG) {
                double dist = Math.abs(candidate - currentAngleCWDeg);
                if (dist < bestDist) {
                    bestDist = dist;
                    best = candidate;
                }
            }
        }

        if (Double.isNaN(best)) {
            // Target genuinely lives in the dead zone - get as close as the hardware allows.
            targetInDeadZone = true;
            double distToZero = Math.abs(0 - currentAngleCWDeg);
            double distToMax = Math.abs(TURRET_TOTAL_TRAVEL_DEG - currentAngleCWDeg);
            return distToZero <= distToMax ? 0 : TURRET_TOTAL_TRAVEL_DEG;
        }

        targetInDeadZone = false;
        return best;
    }

    // clockwise angle between TURRETS pos and target
    public double robotRelativeBearingRadTo(Pose target) {
        if (follower == null) return 0;
        Pose robotPose = follower.pose();
        double heading = robotPose.heading();

        double turretFieldX =
                robotPose.x() +
                        TURRET_OFFSET_FORWARD_IN * Math.cos(heading) -
                        TURRET_OFFSET_LATERAL_IN * Math.sin(heading);
        double turretFieldY =
                robotPose.y() +
                        TURRET_OFFSET_FORWARD_IN * Math.sin(heading) +
                        TURRET_OFFSET_LATERAL_IN * Math.cos(heading);

        double bearingFieldRad = Math.atan2(target.y() - turretFieldY, target.x() - turretFieldX);
        return normalizeRadians(bearingFieldRad - heading);
    }

    public double computeServoPositionForFieldPoint(Pose target) {
        double robotRelativeCCWDeg = Math.toDegrees(robotRelativeBearingRadTo(target));
        double rawAngleCWDeg = normalizeDegrees0to360(-robotRelativeCCWDeg);
        double resolvedAngleCWDeg = resolveWraparound(rawAngleCWDeg, turretAngleCWDeg);
        return angleCWDegToServoPos(resolvedAngleCWDeg);
    }
    ///
    /// Tracking Functions
    ///
    public void setTrackTarget(Pose target) {
        trackTarget = target;
    }

    public void enableTracking() {
        tracking = true;
    }

    public void disableTracking() {
        tracking = false;
    }

    //set the target AND start tracking it in one call
    public void trackTarget(Pose target) {
        setTrackTarget(target);
        enableTracking();
    }
    ///
    /// Relocalize
    ///
    // Snaps the robots pose to a known field position (like lined up against a wall)
    public void relocalizeTo(Pose pose) {
        if (follower != null) {
            follower.setPose(pose);
        }
    }

    public void relocalizeToRedGarden(){
        relocalizeTo(RELOCALIZATION_RED_GARDEN);
    }

    public void relocalizeToBlueGarden(){
        relocalizeTo(RELOCALIZATION_BLUE_GARDEN);
    }

    public void swapHive() {
        hiveAudienceSide = !hiveAudienceSide;
        if (hiveAudienceSide) {
            if (OpModeStorage.red) {
                setTrackTarget(RED_HIVE_GARDEN);
            } else {
                setTrackTarget(BLUE_HIVE_GARDEN);
            }
        } else {
            if (OpModeStorage.red) {
                setTrackTarget(RED_HIVE_FLOWER);
            } else {
                setTrackTarget(BLUE_HIVE_FLOWER);
            }
        }
    }
    ///
    /// Big Daddy
    ///
    public Command periodic() {
        return infinite(()-> {
            trackTargetX = trackTarget.x();
            trackTargetY = trackTarget.y();

            if (tracking && follower != null) {
                turretToPos(computeServoPositionForFieldPoint(trackTarget));
            }

            telemetry.addData("Target X", trackTargetX);
            telemetry.addData("Target Y", trackTargetY);
            telemetry.addData("Tracked Position", computeServoPositionForFieldPoint(trackTarget));
            telemetry.addData("tracking",tracking);
        });
    }
}
