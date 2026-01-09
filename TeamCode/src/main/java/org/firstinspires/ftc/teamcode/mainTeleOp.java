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

        //once you press play
        waitForStart();
        boolean pressingb = false;
        boolean pressingx = true;
        // boolean pressingy = false;

        boolean difference = false;
        while (opModeIsActive()) {

            double leftStickY = -gamepad1.left_stick_y; // Remember, this is reversed!
            double leftStickX = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rightStickX = gamepad1.right_stick_x;

            telemetry.addData("leftStick X", leftStickX);
            telemetry.addData("leftStick Y", leftStickY);
            telemetry.addData("rightStick X", rightStickX);
            telemetry.update();

            drive(leftStickY, leftStickX, rightStickX);
//            initServo();
//            shoot();'
            removeBalls();
            initIntake();
            prepareLaunch();

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

    public void shoot() {
        // will initialize rs and ls with clicking A
        if (!gamepad1.a) {
            robot.rs.setPower(0);
            robot.ls.setPower(0);
        } else {
            robot.rs.setPower(1);
            robot.ls.setPower(1);
        }
    }

    public void initServo() {
        if (!gamepad1.b) {
            robot.demoServo1.setPosition(0.5);
            robot.demoServo2.setPosition(0.5);
        } else {
            robot.demoServo1.setPosition(1);
            robot.demoServo2.setPosition(1);
        }
    }

    //intake code
    public void initIntake() {
        if (!gamepad1.right_bumper) {
            robot.it.setPower(0);
        } else {
            robot.it.setPower(1);
            robot.demoServo1.setPosition(0.25);
            robot.demoServo2.setPosition(0.25);
        }
    }

    public void removeBalls() {
        if (!gamepad1.left_bumper) {
            robot.it.setPower(0);
        } else {
            robot.it.setPower(-1);
            robot.demoServo1.setPosition(0.25);
            robot.demoServo2.setPosition(0.25);
        }
    }


    public void prepareLaunch(){
        if(!gamepad1.y){
            robot.rs.setPower(0);
            robot.ls.setPower(0);
            robot.it.setPower(0);
            robot.demoServo1.setPosition(0.5);
            robot.demoServo2.setPosition(0.5);
        } else{
            robot.rs.setPower(-0.6);
            robot.ls.setPower(-0.6);
            sleep(1500);
            robot.it.setPower(1);
            robot.demoServo1.setPosition(0.75);
            sleep(516);
            robot.demoServo2.setPosition(0.75);
        }
    }
}
