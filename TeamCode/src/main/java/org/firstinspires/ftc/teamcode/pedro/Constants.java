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

    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("left_front");
                c.backLeftName.set("left_back");
                c.frontRightName.set("right_front");
                c.backRightName.set("right_back");

                c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.867);
        c.yPodOffset.set(-3.75);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

//    public static ForesightConfig foresightConfig = new ForesightConfig(
//            c -> {
//                Controller primaryTranslationalForward = Controller.proportional(0.1653582918956712);
//                Controller secondaryTranslationalForward = Controller.proportional(0.061095470400515196);
//                Controller primaryTranslationalLateral = Controller.proportional(0.2211773167045044);
//                Controller secondaryTranslationalLateral = Controller.proportional(0.08171910855556662);
//
//                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//                c.coast.set(Controller.proportionalFeedforward(0.015011838660442966));
//                c.brake.set(Controller.proportionalFeedforward(0.01276006286137652));
//
//                c.headingFeedback.set(Controller.proportional(2.8159952207051564));
//                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05088754513323334, 0.004365789915576249));
//
//                c.linearBrakeCoefficients.set(Matrix.diag(0.0580213792090035, 1.1572231780672053));
//                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0013119945910094917, -0.018287090565586934));
//
//                c.maxAchievableForwardVelocity.set(69.2656689482408);
//                c.maxAchievableStrafeVelocity.set(62.372447645854606);
//                c.naturalForwardDeceleration.set(39.763321343506945);
//                c.naturalStrafeDeceleration.set(55.06295028639528);
//            }
//    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.25903069882504787);
                Controller secondaryTranslationalForward = Controller.proportional(0.09570492178810883);
                Controller primaryTranslationalLateral = Controller.proportional(0.3568264201643619);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1318378276731841);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016312709536176973));
                c.brake.set(Controller.proportionalFeedforward(0.013865803105750427));

                c.headingFeedback.set(Controller.proportional(3.5609563623048546));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05789573163896061, 0.0025314343944571414));

                c.linearBrakeCoefficients.set(Matrix.diag(0.1508250061304573, 0.05613272916055212));
                c.quadraticBrakeCoefficients.set(Matrix.diag(-9.411630778327009E-4, 0.0017122054429979641));

                c.maxAchievableForwardVelocity.set(65.71891899479);
                c.maxAchievableStrafeVelocity.set(54.8310150624043);
                c.naturalForwardDeceleration.set(30.518915970675902);
                c.naturalStrafeDeceleration.set(59.14795151072733);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}