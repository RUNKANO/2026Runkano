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
                Controller primaryTranslationalForward = Controller.proportional(0.2135343977175893);
                Controller secondaryTranslationalForward = Controller.proportional(0.07889525421245804);
                Controller primaryTranslationalLateral = Controller.proportional(0.3246205625695646);
                Controller secondaryTranslationalLateral = Controller.proportional(0.11993862384827052);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.018004855508912173));
                c.brake.set(Controller.proportionalFeedforward(0.015304127182575346));

                c.headingFeedback.set(Controller.proportional(2.528120672040379));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04041117302601002, 0.008368242311015713));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0318999826850604, 0.05690499962733397));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0031124466023202554, 0.0020423710445021436));

                c.maxAchievableForwardVelocity.set(58.57728840205717);
                c.maxAchievableStrafeVelocity.set(48.9764122542386);
                c.naturalForwardDeceleration.set(40.38186358043531);
                c.naturalStrafeDeceleration.set(59.49527769098939);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-2.711741530050443);
        c.yPodOffset.set(-5.77421173335999);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
    c.frontLeftName.set("fL");
    c.frontRightName.set("fR");
    c.backLeftName.set("bL");
    c.backRightName.set("bR");
    c.manualBrakeMode.set(true);
    c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
    c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
    c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
    c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
});
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
    }
