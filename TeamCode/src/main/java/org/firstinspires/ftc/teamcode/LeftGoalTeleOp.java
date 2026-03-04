package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "LeftGoalTeleOp")
public class LeftGoalTeleOp extends LinearOpMode {

    Hardware robot = Hardware.getInstance();
    private Follower follower;

    // Shooter
    private static final double TARGET_VELOCITY = 1700;
    private static final double READY_THRESHOLD = 0.90;
    private static final long READY_TIME_MS = 75;
    private long readyStartTime = 0;
    private boolean flywheelReady = false;

    // Auto-align
    private static final double GOAL_X = 10;
    private static final double GOAL_Y = 138;
    private static final double kP = 0.6;
    private static final double MAX_TURN = 0.4;
    private static final double ALIGN_THRESHOLD = Math.toRadians(0.5);
    private boolean headingLocked = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        if (Hardware.lastAutoPose != null) {
            follower.setPose(Hardware.lastAutoPose);
        }

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        follower.startTeleopDrive(true);

        while (opModeIsActive()) {
            follower.update();

            double forward = -gamepad1.left_stick_y;
            double strafe  =  gamepad1.left_stick_x * 1.1;
            double turn    =  gamepad1.right_stick_x;

            if (gamepad1.x) {
                turn = getAlignTurn();
                if (headingLocked) gamepad1.rumble(200);
            } else {
                headingLocked = false;
            }

            drive(strafe, forward, turn);
            handleIntake();
            handleShooter(gamepad1.y);

            telemetry.addData("Distance",  "%.1f", getDistanceToGoal());
            telemetry.addData("Locked",    headingLocked);
            telemetry.addData("FW Ready",  flywheelReady);
            telemetry.addData("RS Vel",    robot.rs.getVelocity());
            telemetry.addData("LS Vel",    robot.ls.getVelocity());
            telemetry.update();
        }
    }

    // -------------------------------------------------------------------------
    // Drive
    // -------------------------------------------------------------------------

    private void drive(double x, double y, double turn) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);
        robot.setPower(
                (y + x + turn) / denominator,
                (y - x - turn) / denominator,
                (y - x + turn) / denominator,
                (y + x - turn) / denominator
        );
    }

    // -------------------------------------------------------------------------
    // Intake
    // -------------------------------------------------------------------------

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
        }
        // removed the else — let shooter handle it when no buttons pressed
    }

    // -------------------------------------------------------------------------
    // Shooter
    // -------------------------------------------------------------------------

    private void handleShooter(boolean shooting) {
        boolean intakeActive = gamepad1.left_bumper || gamepad1.right_bumper || gamepad1.dpad_up;

        if (!shooting) {
            robot.rs.setVelocity(0);
            robot.ls.setVelocity(0);
            if (!intakeActive) {
                robot.it.setPower(0);
                robot.demoServo1.setPosition(0.5);
            }
            readyStartTime = 0;
            flywheelReady = false;
            return;
        }

        robot.rs.setVelocity(-TARGET_VELOCITY);
        robot.ls.setVelocity(-TARGET_VELOCITY);

        boolean atSpeed =
                Math.abs(robot.rs.getVelocity()) > TARGET_VELOCITY * READY_THRESHOLD &&
                        Math.abs(robot.ls.getVelocity()) > TARGET_VELOCITY * READY_THRESHOLD;

        if (!flywheelReady) {
            if (atSpeed) {
                if (readyStartTime == 0) readyStartTime = System.currentTimeMillis();
                if (System.currentTimeMillis() - readyStartTime > READY_TIME_MS) {
                    flywheelReady = true;
                }
            } else {
                readyStartTime = 0;
            }
        }

        if (!intakeActive) { // only control intake motor if driver isn't using it
            if (flywheelReady) {
                robot.it.setPower(1);
                robot.demoServo1.setPosition(0.75);
            } else {
                robot.it.setPower(0);
                robot.demoServo1.setPosition(0.5);
            }
        }
    }

    // -------------------------------------------------------------------------
    // Auto-align
    // -------------------------------------------------------------------------

    private double getAlignTurn() {
        Pose pose = follower.getPose();
        double dx = GOAL_X - pose.getX();
        double dy = GOAL_Y - pose.getY();

        double targetAngle = Math.atan2(dy, dx) + Math.PI;
        double error = angleWrap(targetAngle - pose.getHeading());

        if (Math.abs(error) < ALIGN_THRESHOLD) {
            headingLocked = true;
            return 0;
        }

        headingLocked = false;
        return Range.clip(kP * error, -MAX_TURN, MAX_TURN);
    }

    private double angleWrap(double angle) {
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // -------------------------------------------------------------------------
    // Utility
    // -------------------------------------------------------------------------

    private double getDistanceToGoal() {
        Pose pose = follower.getPose();
        double dx = GOAL_X - pose.getX();
        double dy = GOAL_Y - pose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
}