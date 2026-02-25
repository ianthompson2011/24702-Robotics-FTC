package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "LeftGoalTeleOp")
public class LeftGoalTeleOp extends LinearOpMode {

    Hardware robot = Hardware.getInstance();
    private Follower follower;

    // Shooter constants
    private static final double HIGH_VELOCITY = 1750;
    private static final double LOW_VELOCITY = 1450;
    private static final double READY_PERCENT = 0.95;

    // Distance shooting tuning
    private static final double BASE_VELOCITY = 1400;
    private static final double DISTANCE_GAIN = 3.0;

    // Right goal position
    private static final double GOAL_X = 10;
    private static final double GOAL_Y = 138;

    private Timer shootTimer = new Timer();
    private boolean autoFeeding = false;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        if (Hardware.lastAutoPose != null) {
            follower.setPose(Hardware.lastAutoPose);
        }

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x * 1.1;
            double turn = gamepad1.right_stick_x;

            // Auto-turn toward goal with back of robot if X is held
            if (gamepad1.x) {
                Pose pose = follower.getPose();
                double dx = GOAL_X - pose.getX();
                double dy = GOAL_Y - pose.getY();
                double goalAngle = Math.atan2(dy, dx) + Math.PI; // rear faces goal

                // Wrap goalAngle to [-π, π]
                while (goalAngle > Math.PI) goalAngle -= 2 * Math.PI;
                while (goalAngle < -Math.PI) goalAngle += 2 * Math.PI;

                double error = goalAngle - pose.getHeading();
                while (error > Math.PI) error -= 2 * Math.PI;
                while (error < -Math.PI) error += 2 * Math.PI;

                double kTurn = 1.2; // reduce gain for stability
                turn = Range.clip(kTurn * error, -0.4, 0.4); // clamp turn to prevent jitter
            }

            drive(strafe, forward, turn);

            handleIntake();
            handleShooter();

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Distance", getDistanceToGoal());
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

        Pose pose = follower.getPose();

        // -------- AUTO DISTANCE + TURN (X) --------
        if (gamepad1.x) {

            double distance = getDistanceToGoal();
            double targetVelocity = BASE_VELOCITY + distance * DISTANCE_GAIN;

            robot.rs.setVelocity(-targetVelocity);
            robot.ls.setVelocity(-targetVelocity);

            boolean ready = Math.abs(robot.rs.getVelocity()) > targetVelocity * READY_PERCENT
                    && Math.abs(robot.ls.getVelocity()) > targetVelocity * READY_PERCENT;

            if (ready) {
                if (!autoFeeding) {
                    shootTimer.resetTimer();
                    autoFeeding = true;
                }
                if (shootTimer.getElapsedTimeSeconds() > 0.25) {
                    robot.it.setPower(1);
                    robot.demoServo1.setPosition(0.75);
                }
            } else {
                autoFeeding = false;
                robot.it.setPower(0);
                robot.demoServo1.setPosition(0.5);
            }

            return;
        }

        autoFeeding = false;

        // -------- MANUAL SHOOTER (Y) --------
        if (!gamepad1.y) {
            robot.rs.setVelocity(0);
            robot.ls.setVelocity(0);
            return;
        }

        double targetVelocity = gamepad1.a ? LOW_VELOCITY : HIGH_VELOCITY;
        robot.rs.setVelocity(-targetVelocity);
        robot.ls.setVelocity(-targetVelocity);

        boolean ready = Math.abs(robot.rs.getVelocity()) > targetVelocity * READY_PERCENT
                && Math.abs(robot.ls.getVelocity()) > targetVelocity * READY_PERCENT;

        if (ready) {
            robot.it.setPower(1);
            robot.demoServo1.setPosition(0.75);
        } else {
            robot.it.setPower(0);
            robot.demoServo1.setPosition(0.5);
        }
    }

    private double getDistanceToGoal() {
        Pose pose = follower.getPose();
        double dx = GOAL_X - pose.getX();
        double dy = GOAL_Y - pose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
}