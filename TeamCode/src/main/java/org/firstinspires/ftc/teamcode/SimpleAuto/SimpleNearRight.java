package org.firstinspires.ftc.teamcode.SimpleAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "SimpleNearRight")
public class SimpleNearRight extends OpMode {

    private Hardware robot = Hardware.getInstance();

    private static final double HIGH_VELOCITY = 1600;
    private static final double READY_PERCENT = 0.95;
    private static final double FEED_TIME     = 0.65;
    private static final double FULL_POWER    = 1.0;

    private boolean shooterSpinning = false;
    private boolean feeding         = false;
    private int     shotsFired      = 0;

    private Follower follower;
    private Timer    pathTimer;
    private Timer    opModeTimer;

    // ── Init config ───────────────────────────────────────────────────────────
    private enum ConfigPhase { MODE_SELECT, DELAY_SELECT, DONE }
    private ConfigPhase configPhase = ConfigPhase.MODE_SELECT;

    private boolean shootMode    = true;
    private int     delaySeconds = 0;

    private boolean dpadUpPrev   = false;
    private boolean dpadDownPrev = false;
    private boolean aButtonPrev  = false;

    // ── Poses ─────────────────────────────────────────────────────────────────
    private final Pose startPose      = new Pose(120.8683, 127.7720, Math.toRadians(40));
    private final Pose shootPose      = new Pose(82,  82,  Math.toRadians(45));
    private final Pose parkAfterShoot = new Pose(93,  125, Math.toRadians(90));  // used after shooting
    private final Pose parkDirectPose = new Pose(85,  130, Math.toRadians(90));  // park-only destination

    private PathChain driveToShootPath;
    private PathChain parkAfterShootPath;
    private PathChain parkDirectPath;
    private boolean pathStarted = false;

    public enum PathState { DELAY, DRIVE_TO_SHOOT, SHOOT_PRELOAD, PARK, IDLE }
    private PathState pathState;

    public void buildPaths() {
        driveToShootPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        parkAfterShootPath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkAfterShoot))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkAfterShoot.getHeading())
                .build();
        parkDirectPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, parkDirectPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), parkDirectPose.getHeading())
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

    public void stateUpdate() {
        switch (pathState) {
            case DELAY:
                if (pathTimer.getElapsedTimeSeconds() >= delaySeconds) {
                    setPathState(shootMode ? PathState.DRIVE_TO_SHOOT : PathState.PARK);
                }
                break;
            case DRIVE_TO_SHOOT:
                if (!pathStarted) {
                    follower.setMaxPower(FULL_POWER);
                    follower.followPath(driveToShootPath, true);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;
            case SHOOT_PRELOAD:
                if (shootThreeBalls()) { setPathState(PathState.PARK); }
                break;
            case PARK:
                if (!pathStarted) {
                    follower.setMaxPower(FULL_POWER);
                    follower.followPath(shootMode ? parkAfterShootPath : parkDirectPath, true);
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
        configPhase = ConfigPhase.MODE_SELECT;
    }

    @Override
    public void init_loop() {
        boolean dpadUpNow   = gamepad1.dpad_up;
        boolean dpadDownNow = gamepad1.dpad_down;
        boolean aButtonNow  = gamepad1.a;

        if (configPhase == ConfigPhase.MODE_SELECT) {
            if ((dpadUpNow && !dpadUpPrev) || (dpadDownNow && !dpadDownPrev)) shootMode = !shootMode;
            if (aButtonNow && !aButtonPrev) configPhase = ConfigPhase.DELAY_SELECT;
            telemetry.addLine("=== STEP 1: SELECT MODE ===");
            telemetry.addData("Mode (dpad up/down)", shootMode ? "SHOOT + PARK" : "PARK ONLY");
            telemetry.addLine("Press A to confirm");

        } else if (configPhase == ConfigPhase.DELAY_SELECT) {
            if (dpadUpNow && !dpadUpPrev)     delaySeconds = Math.min(delaySeconds + 1, 29);
            if (dpadDownNow && !dpadDownPrev) delaySeconds = Math.max(delaySeconds - 1, 0);
            if (aButtonNow && !aButtonPrev)   configPhase = ConfigPhase.DONE;
            telemetry.addLine("=== STEP 2: SET DELAY ===");
            telemetry.addData("Mode locked", shootMode ? "SHOOT + PARK" : "PARK ONLY");
            telemetry.addData("Delay (dpad up/down)", delaySeconds + "s");
            telemetry.addLine("Press A to confirm");

        } else {
            telemetry.addLine("=== READY TO START ===");
            telemetry.addData("Mode",  shootMode ? "SHOOT + PARK" : "PARK ONLY");
            telemetry.addData("Delay", delaySeconds + "s");
        }

        dpadUpPrev   = dpadUpNow;
        dpadDownPrev = dpadDownNow;
        aButtonPrev  = aButtonNow;
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        if (delaySeconds > 0) {
            pathState = PathState.DELAY;
        } else {
            pathState = shootMode ? PathState.DRIVE_TO_SHOOT : PathState.PARK;
        }
    }

    @Override
    public void loop() {
        follower.update();
        stateUpdate();
        telemetry.addData("State",   pathState);
        telemetry.addData("Mode",    shootMode ? "SHOOT + PARK" : "PARK ONLY");
        telemetry.addData("Delay",   delaySeconds + "s");
        telemetry.addData("Shots",   shotsFired);
        telemetry.addData("RS Vel",  String.format("%.0f", robot.rs.getVelocity()));
        telemetry.addData("LS Vel",  String.format("%.0f", robot.ls.getVelocity()));
        telemetry.addData("X",       String.format("%.1f", follower.getPose().getX()));
        telemetry.addData("Y",       String.format("%.1f", follower.getPose().getY()));
        telemetry.addData("Heading", String.format("%.1f", Math.toDegrees(follower.getPose().getHeading())));
        telemetry.update();
    }
}