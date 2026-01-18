package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "mainTeleOp")

public class mainTeleOp extends LinearOpMode {
    Hardware robot = Hardware.getInstance();

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
            prepareLaunch();

            // all methods are defined below
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

    // intake code
    public void initIntake() {
        if (!gamepad1.right_bumper) {
            robot.it.setPower(0);
            // make sure rubber bands don't move without input
        } else {
            // move motors for rubber bands, move servos up
            robot.it.setPower(1);
            robot.demoServo1.setPosition(0.25);
            // prevent balls from falling out by reversing shooting motors
            robot.rs.setPower(1);
            robot.ls.setPower(1);
        }
    }

    // if balls get stuck during intake, this method moves them back out as a wost case scenario
    // effectively the opposite of initIntake()
    public void removeBalls() {
        if (!gamepad1.left_bumper) {
            robot.it.setPower(0);
        } else {
            robot.it.setPower(-1);
            robot.demoServo1.setPosition(0.25);
            robot.rs.setPower(1);
            robot.ls.setPower(1);
        }
    }
    public void prepareLaunch(){
        if(!gamepad1.y){
            // make sure nothing is moving without driver input
            robot.rs.setPower(0);
            robot.ls.setPower(0);
            robot.it.setPower(0);
            robot.demoServo1.setPosition(0.5);
        } else{
            // if pressing x, shooting power will lower slightly
            robot.rs.setPower(gamepad1.x ? -0.55 : -0.6);
            robot.ls.setPower(gamepad1.x ? -0.55 : -0.6);
            // allow motors to reach max speed before moving balls
            sleep(1250);

            robot.it.setPower(1);
            sleep(1500);
            // begin moving rubber bands to move balls up

            robot.demoServo1.setPosition(0.75);
            sleep(750);
            // move servos to get balls prepared for shot

            robot.demoServo1.setPosition(0.25);
            sleep(500);
            // move servos backwards to prevent balls from moving out too fast

            // stop servos to stop entire method
            robot.demoServo1.setPosition(0.5);
        }
    }
}
