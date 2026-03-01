package org.firstinspires.ftc.teamcode.FullAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "NearGoalLeft")
public class NearGoalLeft extends OpMode {

    private Hardware robot = Hardware.getInstance();

    private static final double HIGH_VELOCITY  = 1700;
    private static final double READY_PERCENT  = 0.95;
    private static final double SETTLE_TIME    = 0.5;
    private static final double FEED_TIME      = 0.65;

    private static final double FULL_POWER     = 1.0;
    private static final double INTAKE_POWER   = 0.45;

    private boolean shooterSpinning = false;
    private boolean feeding         = false;
    private int     shotsFired      = 0;

    private Follower follower;
    private Timer    pathTimer;
    private Timer    opModeTimer;

    // ── Poses ─────────────────────────────────────────────────────────────────

    private final Pose startPose    = new Pose(24, 126, Math.toRadians(135));
    private final Pose shootPose    = new Pose(61,  81,  Math.toRadians(135));

    // Row 1 (bottom) — X+2, Y+1.5
    private final Pose row1Start    = new Pose(44,  35.5, Math.toRadians(0));
    private final Pose row1End      = new Pose(18,  35.5, Math.toRadians(0));

    // Intermediate after row 1 — X-15
    private final Pose afterRow1Mid = new Pose(57, 68, Math.toRadians(135));

    // Row 2 (middle) — X+2, Y+1.5
    private final Pose row2Start    = new Pose(44,  58.5, Math.toRadians(0));
    private final Pose row2End      = new Pose(18,  58.5, Math.toRadians(0));

    // Intermediate after row 2 — X-15
    private final Pose afterRow2Mid = new Pose(46, 81, Math.toRadians(135));

    // Row 3 (top) — X+2, Y+1.5
    private final Pose row3Start    = new Pose(44,  85.5, Math.toRadians(0));
    private final Pose row3End      = new Pose(18,  85.5, Math.toRadians(0));

    private final Pose endPose      = new Pose(56, 122, Math.toRadians(90));

    // ─────────────────────────────────────────────────────────────────────────

    private PathChain driveToShootPath;
    private PathChain alignToRow1Path;
    private PathChain intakeRow1Path;
    private PathChain returnAfterRow1Path;
    private PathChain alignToRow2Path;
    private PathChain intakeRow2Path;
    private PathChain returnAfterRow2Path;
    private PathChain alignToRow3Path;
    private PathChain intakeRow3Path;
    private PathChain returnAfterRow3Path;
    private PathChain parkPath;

    private boolean pathStarted = false;

    public enum PathState {
        DRIVE_TO_SHOOT,
        SHOOT_PRELOAD,
        ALIGN_TO_ROW1,
        INTAKE_ROW1,
        RETURN_AFTER_ROW1,
        WAIT_SETTLE_1,
        SHOOT_AFTER_ROW1,
        ALIGN_TO_ROW2,
        INTAKE_ROW2,
        RETURN_AFTER_ROW2,
        WAIT_SETTLE_2,
        SHOOT_AFTER_ROW2,
        ALIGN_TO_ROW3,
        INTAKE_ROW3,
        RETURN_AFTER_ROW3,
        WAIT_SETTLE_3,
        SHOOT_AFTER_ROW3,
        PARK,
        IDLE
    }
    private PathState pathState;

    public void buildPaths() {
        driveToShootPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        alignToRow1Path = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, row1Start))
                .setLinearHeadingInterpolation(shootPose.getHeading(), row1Start.getHeading())
                .build();

        intakeRow1Path = follower.pathBuilder()
                .addPath(new BezierLine(row1Start, row1End))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        returnAfterRow1Path = follower.pathBuilder()
                .addPath(new BezierLine(row1End, afterRow1Mid))
                .setLinearHeadingInterpolation(row1End.getHeading(), afterRow1Mid.getHeading())
                .addPath(new BezierLine(afterRow1Mid, shootPose))
                .setLinearHeadingInterpolation(afterRow1Mid.getHeading(), shootPose.getHeading())
                .build();

        alignToRow2Path = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, row2Start))
                .setLinearHeadingInterpolation(shootPose.getHeading(), row2Start.getHeading())
                .build();

        intakeRow2Path = follower.pathBuilder()
                .addPath(new BezierLine(row2Start, row2End))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        returnAfterRow2Path = follower.pathBuilder()
                .addPath(new BezierLine(row2End, afterRow2Mid))
                .setLinearHeadingInterpolation(row2End.getHeading(), afterRow2Mid.getHeading())
                .addPath(new BezierLine(afterRow2Mid, shootPose))
                .setLinearHeadingInterpolation(afterRow2Mid.getHeading(), shootPose.getHeading())
                .build();

        alignToRow3Path = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, row3Start))
                .setLinearHeadingInterpolation(shootPose.getHeading(), row3Start.getHeading())
                .build();

        intakeRow3Path = follower.pathBuilder()
                .addPath(new BezierLine(row3Start, row3End))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        returnAfterRow3Path = follower.pathBuilder()
                .addPath(new BezierLine(row3End, shootPose))
                .setLinearHeadingInterpolation(row3End.getHeading(), shootPose.getHeading())
                .build();

        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    private boolean shootThreeBalls() {
        if (!shooterSpinning) {
            robot.rs.setVelocity(-HIGH_VELOCITY);
            robot.ls.setVelocity(-HIGH_VELOCITY);
            shooterSpinning = true;
            pathTimer.resetTimer();
            return false;
        }

        if (shotsFired >= 3) {
            robot.rs.setVelocity(0);
            robot.ls.setVelocity(0);
            return true;
        }

        if (!feeding) {
            boolean ready = Math.abs(robot.rs.getVelocity()) > HIGH_VELOCITY * READY_PERCENT
                    && Math.abs(robot.ls.getVelocity()) > HIGH_VELOCITY * READY_PERCENT;
            if (!ready) return false;
            robot.it.setPower(1);
            robot.demoServo1.setPosition(0.75);
            feeding = true;
            pathTimer.resetTimer();
        } else {
            if (pathTimer.getElapsedTimeSeconds() > FEED_TIME) {
                robot.it.setPower(0);
                robot.demoServo1.setPosition(0.5);
                feeding = false;
                shotsFired++;
                pathTimer.resetTimer();
                robot.rs.setVelocity(-HIGH_VELOCITY);
                robot.ls.setVelocity(-HIGH_VELOCITY);
            }
        }
        return false;
    }

    private void resetShooter() {
        shooterSpinning = false;
        feeding         = false;
        shotsFired      = 0;
    }

    private void startReturnWithPrespin(PathChain path) {
        follower.setMaxPower(FULL_POWER);
        robot.rs.setVelocity(-HIGH_VELOCITY);
        robot.ls.setVelocity(-HIGH_VELOCITY);
        shooterSpinning = true;
        follower.followPath(path, true);
        pathStarted = true;
    }

    public void stateUpdate() {
        switch (pathState) {

            case DRIVE_TO_SHOOT:
                if (!pathStarted) {
                    follower.setMaxPower(FULL_POWER);
                    follower.followPath(driveToShootPath, true);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    resetShooter();
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                if (shootThreeBalls()) { resetShooter(); setPathState(PathState.ALIGN_TO_ROW1); }
                break;

            case ALIGN_TO_ROW1:
                if (!pathStarted) {
                    follower.setMaxPower(FULL_POWER);
                    follower.followPath(alignToRow1Path, true);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    setPathState(PathState.INTAKE_ROW1);
                }
                break;

            case INTAKE_ROW1:
                if (!pathStarted) {
                    follower.setMaxPower(INTAKE_POWER);
                    follower.followPath(intakeRow1Path, true);
                    robot.it.setPower(1);
                    robot.demoServo1.setPosition(0);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    robot.it.setPower(0);
                    robot.demoServo1.setPosition(0.5);
                    setPathState(PathState.RETURN_AFTER_ROW1);
                }
                break;

            case RETURN_AFTER_ROW1:
                if (!pathStarted) {
                    startReturnWithPrespin(returnAfterRow1Path);
                } else if (!follower.isBusy()) {
                    shotsFired = 0;
                    feeding = false;
                    setPathState(PathState.WAIT_SETTLE_1);
                }
                break;

            case WAIT_SETTLE_1:
                if (pathTimer.getElapsedTimeSeconds() > SETTLE_TIME) { setPathState(PathState.SHOOT_AFTER_ROW1); }
                break;

            case SHOOT_AFTER_ROW1:
                if (shootThreeBalls()) { resetShooter(); setPathState(PathState.ALIGN_TO_ROW2); }
                break;

            case ALIGN_TO_ROW2:
                if (!pathStarted) {
                    follower.setMaxPower(FULL_POWER);
                    follower.followPath(alignToRow2Path, true);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    setPathState(PathState.INTAKE_ROW2);
                }
                break;

            case INTAKE_ROW2:
                if (!pathStarted) {
                    follower.setMaxPower(INTAKE_POWER);
                    follower.followPath(intakeRow2Path, true);
                    robot.it.setPower(1);
                    robot.demoServo1.setPosition(0);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    robot.it.setPower(0);
                    robot.demoServo1.setPosition(0.5);
                    setPathState(PathState.RETURN_AFTER_ROW2);
                }
                break;

            case RETURN_AFTER_ROW2:
                if (!pathStarted) {
                    startReturnWithPrespin(returnAfterRow2Path);
                } else if (!follower.isBusy()) {
                    shotsFired = 0;
                    feeding = false;
                    setPathState(PathState.WAIT_SETTLE_2);
                }
                break;

            case WAIT_SETTLE_2:
                if (pathTimer.getElapsedTimeSeconds() > SETTLE_TIME) { setPathState(PathState.SHOOT_AFTER_ROW2); }
                break;

            case SHOOT_AFTER_ROW2:
                if (shootThreeBalls()) { resetShooter(); setPathState(PathState.ALIGN_TO_ROW3); }
                break;

            case ALIGN_TO_ROW3:
                if (!pathStarted) {
                    follower.setMaxPower(FULL_POWER);
                    follower.followPath(alignToRow3Path, true);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    setPathState(PathState.INTAKE_ROW3);
                }
                break;

            case INTAKE_ROW3:
                if (!pathStarted) {
                    follower.setMaxPower(INTAKE_POWER);
                    follower.followPath(intakeRow3Path, true);
                    robot.it.setPower(1);
                    robot.demoServo1.setPosition(0);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    robot.it.setPower(0);
                    robot.demoServo1.setPosition(0.5);
                    setPathState(PathState.RETURN_AFTER_ROW3);
                }
                break;

            case RETURN_AFTER_ROW3:
                if (!pathStarted) {
                    startReturnWithPrespin(returnAfterRow3Path);
                } else if (!follower.isBusy()) {
                    shotsFired = 0;
                    feeding = false;
                    setPathState(PathState.WAIT_SETTLE_3);
                }
                break;

            case WAIT_SETTLE_3:
                if (pathTimer.getElapsedTimeSeconds() > SETTLE_TIME) { setPathState(PathState.SHOOT_AFTER_ROW3); }
                break;

            case SHOOT_AFTER_ROW3:
                if (shootThreeBalls()) { resetShooter(); setPathState(PathState.PARK); }
                break;

            case PARK:
                if (!pathStarted) {
                    follower.setMaxPower(FULL_POWER);
                    follower.followPath(parkPath, true);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    setPathState(PathState.IDLE);
                }
                break;

            case IDLE:
                robot.it.setPower(0);
                Hardware.lastAutoPose = follower.getPose();
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState   = newState;
        pathTimer.resetTimer();
        pathStarted = false;
    }

    @Override
    public void init() {
        robot.init(hardwareMap);
        follower    = Constants.createFollower(hardwareMap);
        pathTimer   = new Timer();
        opModeTimer = new Timer();
        buildPaths();
        follower.setPose(startPose);
        pathState = PathState.DRIVE_TO_SHOOT;
    }

    @Override
    public void start() { opModeTimer.resetTimer(); }

    @Override
    public void loop() {
        follower.update();
        stateUpdate();
        telemetry.addData("State",   pathState);
        telemetry.addData("Shots",   shotsFired);
        telemetry.addData("Feeding", feeding);
        telemetry.addData("RS Vel",  String.format("%.0f", robot.rs.getVelocity()));
        telemetry.addData("LS Vel",  String.format("%.0f", robot.ls.getVelocity()));
        telemetry.addData("X",       String.format("%.1f", follower.getPose().getX()));
        telemetry.addData("Y",       String.format("%.1f", follower.getPose().getY()));
        telemetry.addData("Heading", String.format("%.1f", Math.toDegrees(follower.getPose().getHeading())));
        telemetry.update();
    }
}