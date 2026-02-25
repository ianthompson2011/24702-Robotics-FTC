package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "NearGoalLeft")
public class NearGoalLeft extends OpMode {

    private Hardware robot = Hardware.getInstance();

    // Shooter tuning (ORIGINAL behavior)
    private static final double HIGH_VELOCITY = 1500;
    private static final double READY_PERCENT = 0.95;
    private static final double SETTLE_TIME = 0.4;
    private static final double MAX_INTAKE_TIME = 2.5;

    private boolean shooterSpinning = false;
    private boolean feeding = false;
    private int shotsFired = 0;
    private boolean intakeOn = false;

    private Follower follower;
    private Timer pathTimer;

    // Cycle control
    private int cycleIndex = 0; // 0,1,2

    public enum PathState {
        DRIVE_TO_SHOOT,
        SHOOT_PRELOAD,
        MOVE_TO_ROW,
        INTAKE_ROW,
        RETURN_TO_SHOOT,
        WAIT_FOR_SETTLE,
        SHOOT_CYCLE,
        IDLE
    }

    private PathState pathState;
    private boolean pathStarted = false;

    // ===== Poses =====
    private final Pose startPose = new Pose(23.37,126.79,Math.toRadians(140));
    private final Pose shootPose = new Pose(55.41,87.38,Math.toRadians(135));

    private final Pose row1Start = new Pose(55.48,62.74,Math.toRadians(0));
    private final Pose row1End   = new Pose(14.53,62.74,Math.toRadians(0));

    private final Pose row2Start = new Pose(43.69,87.91,Math.toRadians(0));
    private final Pose row2End   = new Pose(12.62,87.91,Math.toRadians(0));

    private final Pose row3Start = new Pose(42.71,37.17,Math.toRadians(0));
    private final Pose row3End   = new Pose(10.28,37.17,Math.toRadians(0));

    // ===== Paths =====
    private PathChain driveToShootPath;
    private PathChain[] moveToRowPaths = new PathChain[3];
    private PathChain[] intakeRowPaths = new PathChain[3];
    private PathChain[] returnPaths = new PathChain[3];

    public void buildPaths() {

        driveToShootPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        Pose[] starts = {row1Start, row2Start, row3Start};
        Pose[] ends   = {row1End, row2End, row3End};

        for (int i = 0; i < 3; i++) {

            moveToRowPaths[i] = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, starts[i]))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), starts[i].getHeading())
                    .build();

            intakeRowPaths[i] = follower.pathBuilder()
                    .addPath(new BezierLine(starts[i], ends[i]))
                    .setLinearHeadingInterpolation(starts[i].getHeading(), ends[i].getHeading())
                    .build();

            returnPaths[i] = follower.pathBuilder()
                    .addPath(new BezierLine(ends[i], shootPose))
                    .setLinearHeadingInterpolation(ends[i].getHeading(), shootPose.getHeading())
                    .build();
        }
    }

    // ===== ORIGINAL Shooting Logic =====
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
                boolean ready =
                        Math.abs(robot.rs.getVelocity()) > HIGH_VELOCITY * READY_PERCENT &&
                                Math.abs(robot.ls.getVelocity()) > HIGH_VELOCITY * READY_PERCENT;

                if (!ready) return false;

                robot.it.setPower(1);
                robot.demoServo1.setPosition(0.75);
                feeding = true;
                pathTimer.resetTimer();
                return false;
            }

            if (feeding && t > 0.45) {
                robot.it.setPower(0);
                robot.demoServo1.setPosition(0.5);
                feeding = false;
                shotsFired++;
                pathTimer.resetTimer();
            }

            return false;
        }

        robot.rs.setVelocity(0);
        robot.ls.setVelocity(0);
        return true;
    }

    private void runIntake() {
        robot.it.setPower(1);
        intakeOn = true;
    }

    private void stopIntake() {
        robot.it.setPower(0);
        intakeOn = false;
    }

    // ===== State Machine =====
    public void stateUpdate() {

        switch (pathState) {

            case DRIVE_TO_SHOOT:
                if (!pathStarted) {
                    follower.followPath(driveToShootPath, true);
                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    resetShooter();
                    cycleIndex = 0;
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                if (shootThreeBalls()) {
                    resetShooter();
                    setPathState(PathState.MOVE_TO_ROW);
                }
                break;

            case MOVE_TO_ROW:
                if (cycleIndex >= 3) {
                    setPathState(PathState.IDLE);
                    break;
                }

                if (!pathStarted) {
                    follower.followPath(moveToRowPaths[cycleIndex], true);
                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    setPathState(PathState.INTAKE_ROW);
                }
                break;

            case INTAKE_ROW:
                if (!pathStarted) {
                    follower.followPath(intakeRowPaths[cycleIndex], true);
                    runIntake();
                    pathTimer.resetTimer();
                    pathStarted = true;
                }

                if (!follower.isBusy() ||
                        pathTimer.getElapsedTimeSeconds() > MAX_INTAKE_TIME) {

                    stopIntake();
                    setPathState(PathState.RETURN_TO_SHOOT);
                }
                break;

            case RETURN_TO_SHOOT:
                if (!pathStarted) {
                    robot.rs.setVelocity(-HIGH_VELOCITY);
                    robot.ls.setVelocity(-HIGH_VELOCITY);
                    shooterSpinning = true;

                    follower.followPath(returnPaths[cycleIndex], true);
                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    shotsFired = 0;
                    feeding = false;
                    pathTimer.resetTimer();
                    setPathState(PathState.WAIT_FOR_SETTLE);
                }
                break;

            case WAIT_FOR_SETTLE:
                if (pathTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setPathState(PathState.SHOOT_CYCLE);
                }
                break;

            case SHOOT_CYCLE:
                if (shootThreeBalls()) {
                    resetShooter();
                    cycleIndex++;
                    setPathState(PathState.MOVE_TO_ROW);
                }
                break;

            case IDLE:
                stopIntake();
                // After all paths complete
                Hardware.lastAutoPose = follower.getPose();
                break;
        }
    }

    private void resetShooter() {
        shooterSpinning = false;
        feeding = false;
        shotsFired = 0;
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        pathStarted = false;
    }

    @Override
    public void init() {
        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        pathTimer = new Timer();

        buildPaths();
        follower.setPose(startPose);

        pathState = PathState.DRIVE_TO_SHOOT;
    }

    @Override
    public void loop() {
        follower.update();
        stateUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("Cycle", cycleIndex);
        telemetry.addData("Shots", shotsFired);
        telemetry.update();
    }
}