package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "LeftGoalTeleOp")
public class LeftGoalTeleOp extends LinearOpMode {

    Hardware robot = Hardware.getInstance();
    private Follower follower;

    private static final double READY_PERCENT = 0.95;
    private static final double GOAL_X = 10;
    private static final double GOAL_Y = 138;
    private static final double REFERENCE_DISTANCE = 24.0;
    private static final double REFERENCE_VELOCITY = 1750.0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Initialize Pedro Follower
        follower = Constants.createFollower(hardwareMap);
        if (Hardware.lastAutoPose != null) {
            follower.setPose(Hardware.lastAutoPose);
        }

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        // Track pose only, do NOT let Follower control motors
        follower.startTeleopDrive(true); // critical change: false disables motor output

        while (opModeIsActive()) {
            follower.update(); // updates internal pose only

            double forward = -gamepad1.left_stick_y;
            double strafe  = gamepad1.left_stick_x * 1.1;
            double turn    = gamepad1.right_stick_x;

            drive(strafe, forward, turn);
            handleIntake();
            handleShooter();

            double dist = getDistanceToGoal();
            telemetry.addData("Distance to Goal (in)", String.format("%.1f", dist));
            telemetry.addData("Target Velocity", String.format("%.0f", distanceToVelocity(dist)));
            telemetry.addData("RS Velocity", robot.rs.getVelocity());
            telemetry.addData("LS Velocity", robot.ls.getVelocity());
            telemetry.update();
        }
    }

    public void drive(double x, double y, double turn) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);
        robot.lf.setPower((y + x + turn) / denominator);
        robot.lb.setPower((y - x + turn) / denominator);
        robot.rf.setPower((y - x - turn) / denominator);
        robot.rb.setPower((y + x - turn) / denominator);
    }

    private void handleIntake() {
        if (gamepad1.left_bumper) {
            robot.it.setPower(-1);
            robot.demoServo1.setPosition(0);
        } else if (gamepad1.right_bumper) {
            robot.it.setPower(1);
            robot.demoServo1.setPosition(0);
        } else if (gamepad1.dpad_up) {
            robot.it.setPower(1);
            robot.demoServo1.setPosition(1);
        } else {
            robot.it.setPower(0);
            robot.demoServo1.setPosition(0.5);
        }
    }

    private void handleShooter() {
        if (!gamepad1.y) {
            robot.rs.setVelocity(0);
            robot.ls.setVelocity(0);
            robot.demoServo1.setPosition(0.5);
            return;
        }

        double targetVelocity = gamepad1.x
                ? distanceToVelocity(getDistanceToGoal())
                : 1500;

        robot.rs.setVelocity(-targetVelocity);
        robot.ls.setVelocity(-targetVelocity);

        boolean shooterReady =
                Math.abs(robot.rs.getVelocity()) > targetVelocity * READY_PERCENT &&
                        Math.abs(robot.ls.getVelocity()) > targetVelocity * READY_PERCENT;

        if (shooterReady) {
            robot.it.setPower(1);
            robot.demoServo1.setPosition(0.75);
        } else {
            robot.it.setPower(0);
            robot.demoServo1.setPosition(0.5);
        }
    }

    private double distanceToVelocity(double distanceInches) {
        return (distanceInches / REFERENCE_DISTANCE) * REFERENCE_VELOCITY;
    }

    private double getDistanceToGoal() {
        Pose pose = follower.getPose();
        double dx = GOAL_X - pose.getX();
        double dy = GOAL_Y - pose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
}