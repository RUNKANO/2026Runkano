package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3492647642820966);
                Controller secondaryTranslationalForward = Controller.proportional(0.12904399787585347);
                Controller primaryTranslationalLateral = Controller.proportional(0.48774374811599724);
                Controller secondaryTranslationalLateral = Controller.proportional(0.18020828217588367);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.014449984774359199));
                c.brake.set(Controller.proportionalFeedforward(0.012282487058205319));

                c.headingFeedback.set(Controller.proportional(5.19949528443109));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.059073300264201704, 0.005319040483931692));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09950651817072094, 0.05619868582782658));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001418340575368303, 0.0016897978771921354));

                c.maxAchievableForwardVelocity.set(72.6905629294977);
                c.maxAchievableStrafeVelocity.set(49.52339690957358);
                c.naturalForwardDeceleration.set(32.73394112781208);
                c.naturalStrafeDeceleration.set(63.48074900836222);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-6.105531707523376);
        c.yPodOffset.set(2.1566297876553273);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fL");
        c.frontRightName.set("fR");
        c.backLeftName.set("bL");
        c.backRightName.set("bR");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
    }
