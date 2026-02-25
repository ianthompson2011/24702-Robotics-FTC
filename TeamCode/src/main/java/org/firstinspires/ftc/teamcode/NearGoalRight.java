package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "NearGoalRight")
public class NearGoalRight extends OpMode {

    private Hardware robot = Hardware.getInstance();

    // Shooter constants
    private static final double HIGH_VELOCITY = 1500;
    private static final double READY_PERCENT = 0.95;

    // Shooter & intake state variables
    private boolean shooterSpinning = false;
    private boolean feeding = false;
    private int shotsFired = 0;
    private boolean intakeOn = false;

    // Pedro follower
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // Path state machine
    public enum PathState {
        DRIVE_TO_SHOOT,
        SHOOT_PRELOAD,
        DRIVE_TO_INTAKE,
        RETURN_TO_SHOOT,
        SHOOT_SECONDARY,
        IDLE
    }
    PathState pathState;

    // Poses
    private final Pose startPose = new Pose(120.67, 126.20, Math.toRadians(40));
    private final Pose shootPose = new Pose(88.63, 87.97, Math.toRadians(45));
    private final Pose intakeStart = new Pose(88.30, 59.28, Math.toRadians(180));
    private final Pose intakeEnd = new Pose(133.01, 59.71, Math.toRadians(180));

    // Paths
    private PathChain driveToShootPath;
    private PathChain intakePath;
    private PathChain returnToShootPath;

    // =========================
    // Build Paths
    // =========================
    public void buildPaths() {
        // Drive to shooting position
        driveToShootPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // Drive to intake position (slow speed)
        intakePath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intakeEnd))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeEnd.getHeading())
                .setVelocityConstraint(0.5)
                .build();

        // Return to shooting position (full speed)
        returnToShootPath = follower.pathBuilder()
                .addPath(new BezierLine(intakeEnd, shootPose))
                .setLinearHeadingInterpolation(intakeEnd.getHeading(), shootPose.getHeading())
                .build();
    }

    // =========================
    // Shooting Method
    // =========================
    private boolean shootThreeBalls() {
        if (!shooterSpinning) {
            robot.rs.setVelocity(-HIGH_VELOCITY);
            robot.ls.setVelocity(-HIGH_VELOCITY);
            shooterSpinning = true;
            pathTimer.resetTimer();
            return false;
        }

        double t = pathTimer.getElapsedTimeSeconds();

        if (shotsFired < 3) {
            if (!feeding) {
                boolean ready = Math.abs(robot.rs.getVelocity()) > HIGH_VELOCITY * READY_PERCENT &&
                        Math.abs(robot.ls.getVelocity()) > HIGH_VELOCITY * READY_PERCENT;
                if (!ready) return false;

                robot.it.setPower(1);
                robot.demoServo1.setPosition(0.75);
                feeding = true;
                pathTimer.resetTimer();
                return false;
            }

            if (feeding && t > 0.5) {
                robot.it.setPower(0);
                robot.demoServo1.setPosition(0.5);
                feeding = false;
                shotsFired++;
                pathTimer.resetTimer();
            }

            return false;
        }

        // Done shooting
        robot.it.setPower(0);
        robot.demoServo1.setPosition(0.5);
        robot.rs.setVelocity(0);
        robot.ls.setVelocity(0);
        return true;
    }

    // =========================
    // Intake Method
    // =========================
    private void runIntake() {
        robot.it.setPower(1);
        intakeOn = true;
    }

    private void stopIntake() {
        robot.it.setPower(0);
        intakeOn = false;
    }

    // =========================
    // State Machine
    // =========================
    public void stateUpdate() {
        switch (pathState) {
            case DRIVE_TO_SHOOT:
                follower.followPath(driveToShootPath, true);
                shooterSpinning = false;
                feeding = false;
                shotsFired = 0;
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    if (shootThreeBalls()) {
                        setPathState(PathState.DRIVE_TO_INTAKE);
                    }
                }
                break;

            case DRIVE_TO_INTAKE:
                follower.followPath(intakePath, true);
                runIntake();
                if (!follower.isBusy()) {
                    stopIntake();
                    setPathState(PathState.RETURN_TO_SHOOT);
                }
                break;

            case RETURN_TO_SHOOT:
                follower.followPath(returnToShootPath, true);
                // reset shooter for second batch
                shooterSpinning = false;
                feeding = false;
                shotsFired = 0;
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_SECONDARY);
                }
                break;

            case SHOOT_SECONDARY:
                if (!follower.isBusy()) {
                    if (shootThreeBalls()) {
                        setPathState(PathState.IDLE);
                    }
                }
                break;

            case IDLE:
                stopIntake();
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    // =========================
    // FTC Lifecycle
    // =========================
    @Override
    public void init() {
        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        pathTimer = new Timer();
        opModeTimer = new Timer();

        buildPaths();
        follower.setPose(startPose);

        pathState = PathState.DRIVE_TO_SHOOT;
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        stateUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("Shots Fired", shotsFired);
        telemetry.addData("Intake On", intakeOn);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
    }
}