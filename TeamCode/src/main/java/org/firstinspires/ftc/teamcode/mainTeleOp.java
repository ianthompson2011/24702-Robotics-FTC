package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "mainTeleOp")

public class mainTeleOp extends LinearOpMode {
    Hardware robot = Hardware.getInstance();
    private static final double HIGH_GOAL_VELOCITY = 2800; // ticks/sec
    private static final double LOW_GOAL_VELOCITY  = 2500; // ticks/sec
    private static final double READY_PERCENT = 0.95; // 95%

    public void runOpMode() {
        //once you press init

        robot.init(hardwareMap);
        telemetry.addData("Status", "Hello, Drivers!");
        telemetry.update();

        // waiting for start until "play" is pressed
        waitForStart();

        while (opModeIsActive()) {

            double leftStickY = -gamepad1.left_stick_y; // Remember, this is reversed!
            double leftStickX = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rightStickX = gamepad1.right_stick_x;

            telemetry.addData("leftStick X", leftStickX);
            telemetry.addData("leftStick Y", leftStickY);
            telemetry.addData("rightStick X", rightStickX);
            telemetry.update();

            drive(leftStickY, leftStickX, rightStickX);
            removeBalls();
            initIntake();
            initIntakeForShot();
            prepareLaunch();

            //debug telemetry
            telemetry.addData("RS Velocity", robot.rs.getVelocity());
            telemetry.addData("LS Velocity", robot.ls.getVelocity());
            telemetry.update();
        }
    }
    public void drive(double x, double y, double strafe) {

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(strafe), 1);
        double frontLeftPower = (y + x + strafe) / denominator;
        double backLeftPower = (y - x + strafe) / denominator;
        double frontRightPower = (y - x - strafe) / denominator;
        double backRightPower = (y + x - strafe) / denominator;

        robot.lf.setPower(frontLeftPower);
        robot.lb.setPower(backLeftPower);
        robot.rf.setPower(frontRightPower);
        robot.rb.setPower(backRightPower);
    }

    // if balls get stuck during intake, this method moves them back out as a wost case scenario
    // effectively the opposite of initIntake()
    public void removeBalls() {
        if (!gamepad1.left_bumper) {
            robot.it.setPower(0);
        } else {
            robot.it.setPower(-1);
            robot.demoServo1.setPosition(0);
        }
    }

    public void initIntake(){
        if (!gamepad1.right_bumper) {
            robot.it.setPower(0);
            // make sure rubber bands don't move without input
        } else {
            // move motors for rubber bands, move servos up
            robot.it.setPower(1);
            robot.demoServo1.setPosition(0);
            // prevent balls from falling out by reversing shooting motors
        }
    }

    // intake code
    public void initIntakeForShot() {
        if (!gamepad1.dpad_up) {
            robot.it.setPower(0);
            // make sure rubber bands don't move without input
        } else {
            // move motors for rubber bands, move servos up
            robot.it.setPower(1);
            robot.demoServo1.setPosition(0.75);
            // prevent balls from falling out by reversing shooting motors
            robot.rs.setVelocity(2500);
            robot.ls.setVelocity(2500);
        }
    }
public void prepareLaunch() {

    if (!gamepad1.y) {
        // make sure nothing is moving without driver input
        robot.rs.setVelocity(0);
        robot.ls.setVelocity(0);
        robot.it.setPower(0);
        robot.demoServo1.setPosition(0.5);

    } else {
        // velocity targets (tune these)
        double HIGH_VELOCITY = 1500;
        double LOW_VELOCITY  = 1200;

        double targetVelocity = gamepad1.x ? LOW_VELOCITY : HIGH_VELOCITY;

        // spin up shooters using encoders
        robot.rs.setVelocity(-targetVelocity);
        robot.ls.setVelocity(-targetVelocity);

        // only feed balls once shooter is up to speed
        boolean shooterReady =
                Math.abs(robot.rs.getVelocity()) > targetVelocity * 0.95 &&
                        Math.abs(robot.ls.getVelocity()) > targetVelocity * 0.95;

        if (shooterReady) {
            robot.it.setPower(1);
            robot.demoServo1.setPosition(0.25);
        } else {
            robot.it.setPower(0);
            robot.demoServo1.setPosition(0.5);
        }
    }

}
}
