package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "RightGoalTeleOp")
public class RightGoalTeleOp extends LinearOpMode {

    Hardware robot = Hardware.getInstance();
    private Follower follower;

    private static final double TARGET_VELOCITY = 1750; // slightly lower
    private static final long READY_TIME_MS = 150;

    private static final double GOAL_X = 133;
    private static final double GOAL_Y = 138;

    // ---- Stable Aim Tuning ----
    private static final double kP = 0.6;               // faster rotation toward goal
    private static final double MAX_TURN = 0.4;         // max turn power
    private static final double ALIGN_THRESHOLD = Math.toRadians(0.5); // tight lock, no oscillation

    private boolean headingLocked = false;
    private long readyStartTime = 0;

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
            double strafe  = gamepad1.left_stick_x * 1.1;
            double turn    = gamepad1.right_stick_x;

            // Hold X to auto-align
            if (gamepad1.x) {
                turn = getAlignTurn();
                // Vibrate controller when fully aligned
                if (headingLocked) gamepad1.rumble(200);
            } else {
                headingLocked = false; // release lock when X released
            }

            drive(strafe, forward, turn);

            handleIntake();
            handleShooter(gamepad1.y);

            telemetry.addData("Distance", "%.1f", getDistanceToGoal());
            telemetry.addData("Locked", headingLocked);
            telemetry.addData("RS Vel", robot.rs.getVelocity());
            telemetry.addData("LS Vel", robot.ls.getVelocity());
            telemetry.update();
        }
    }

    private double getAlignTurn() {
        Pose pose = follower.getPose();

        double dx = GOAL_X - pose.getX();
        double dy = GOAL_Y - pose.getY();

        double targetAngle = Math.atan2(dy, dx) + Math.PI; // front backwards
        double error = angleWrap(targetAngle - pose.getHeading());

        if (Math.abs(error) < ALIGN_THRESHOLD) {
            headingLocked = true;
            return 0; // stop turning when aligned
        }

        headingLocked = false;
        double output = kP * error;
        return Range.clip(output, -MAX_TURN, MAX_TURN);
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public void drive(double x, double y, double turn) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);
        robot.setPower(
                (y + x + turn) / denominator,
                (y - x - turn) / denominator,
                (y - x + turn) / denominator,
                (y + x - turn) / denominator
        );
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

    private void handleShooter(boolean shooting) {
        if (!shooting) {
            robot.rs.setVelocity(0);
            robot.ls.setVelocity(0);
            robot.demoServo1.setPosition(0.5);
            readyStartTime = 0;
            return;
        }

        robot.rs.setVelocity(-TARGET_VELOCITY);
        robot.ls.setVelocity(-TARGET_VELOCITY);

        boolean ready =
                Math.abs(robot.rs.getVelocity()) > TARGET_VELOCITY * 0.95 &&
                        Math.abs(robot.ls.getVelocity()) > TARGET_VELOCITY * 0.95;

        if (ready) {
            if (readyStartTime == 0) readyStartTime = System.currentTimeMillis();
            if (System.currentTimeMillis() - readyStartTime > READY_TIME_MS) {
                robot.it.setPower(1);
                robot.demoServo1.setPosition(0.75);
            }
        } else {
            readyStartTime = 0;
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