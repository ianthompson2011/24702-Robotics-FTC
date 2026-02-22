package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static PathConstraints pathConstraints = new PathConstraints(0.99,
                                                        100,
                                                        1.5,
                                                        5);

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.25) // update the mass in kilos for the robot when complete!!!
            .forwardZeroPowerAcceleration(-26)
            .lateralZeroPowerAcceleration(-51)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0.0001, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0, 0.002, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.04, 0, 0.00001, 0.6, 0.01))
            .centripetalScaling(0.00045);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("cm2")
            .rightFrontMotorName("cm3")
            .rightRearMotorName("cm0")
            .leftRearMotorName("cm1")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(72)
            .yVelocity(64);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.625) // both must be in inches
            .strafePodX(3.25) // if pedro pathing does not work, this might be why, ensure proper measurements
            .distanceUnit(DistanceUnit.INCH)
            // Make sure to replace the pinpoint hardware map name with the actual name.
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}

