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
                Controller primaryTranslationalForward = Controller.proportional(0.3102708099787026);
                Controller secondaryTranslationalForward = Controller.proportional(0.11463677369839802);
                Controller primaryTranslationalLateral = Controller.proportional(0.4576669574821946);
                Controller secondaryTranslationalLateral = Controller.proportional(0.16909571170334067);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012915799142970403));
                c.brake.set(Controller.proportionalFeedforward(0.010978429271524842));

                c.headingFeedback.set(Controller.proportional(5.4164402998976495));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.059052997293989086, 0.004871298625545696));

                c.linearBrakeCoefficients.set(Matrix.diag(0.12156973052171101, 0.09194558399390296));
                c.quadraticBrakeCoefficients.set(Matrix.diag(4.732403394476893E-4, 7.256817958379476E-4));

                c.maxAchievableForwardVelocity.set(74.37713246257495);
                c.maxAchievableStrafeVelocity.set(58.854703409926834);
                c.naturalForwardDeceleration.set(47.11753261379066);
                c.naturalStrafeDeceleration.set(71.37957312473426);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-6.105531707523376);
        c.yPodOffset.set(2.1566297876553273);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
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
