package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {

    Hardware robot = Hardware.getInstance();

    // shooter velocities (ticks/sec)
    private static final double HIGH_VELOCITY = 1500;
    private static final double LOW_VELOCITY  = 1400;
    private static final double READY_PERCENT = 0.95;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double forward = -gamepad1.left_stick_y;
            double strafe  = gamepad1.left_stick_x * 1.1;
            double turn    = gamepad1.right_stick_x;

            drive(strafe, forward, turn);

            // ---------------- INTAKE ----------------
            handleIntake();

            // ---------------- SHOOTER ----------------
            handleShooter();
        }
    }

    // ---------------- DRIVE ----------------
    public void drive(double x, double y, double turn) {

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);
        double frontLeftPower = (y + x + turn) / denominator;
        double backLeftPower = (y - x + turn) / denominator;
        double frontRightPower = (y - x - turn) / denominator;
        double backRightPower = (y + x - turn) / denominator;

        robot.lf.setPower(frontLeftPower);
        robot.lb.setPower(backLeftPower);
        robot.rf.setPower(frontRightPower);
        robot.rb.setPower(backRightPower);
    }

    // ---------------- INTAKE ONLY ----------------
    private void handleIntake() {

        if (gamepad1.left_bumper) {
            // reverse intake (unstick balls)
            robot.it.setPower(-1);
            robot.demoServo1.setPosition(0);
        } else if (gamepad1.right_bumper) {
            // normal intake
            robot.it.setPower(1);
            robot.demoServo1.setPosition(0);
        } else if (gamepad1.dpad_up){
            // ready for shooting
            robot.it.setPower(1);
            robot.demoServo1.setPosition(1);
        } else {
            robot.it.setPower(0);
            robot.demoServo1.setPosition(0.5);
        }
    }

    // ---------------- SHOOTER ONLY ----------------
    private void handleShooter() {

        if (!gamepad1.y) {
            // shooter OFF
            robot.rs.setVelocity(0);
            robot.ls.setVelocity(0);
            robot.demoServo1.setPosition(0.5);
            return;
        }

        // choose velocity
        double targetVelocity = gamepad1.x ? LOW_VELOCITY : HIGH_VELOCITY;

        // NOTE:
        // rs is REVERSED in hardware
        // ls is FORWARD
        // both use POSITIVE velocity
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
}
