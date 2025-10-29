package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "demoTeleop")

public class mainTeleOp extends LinearOpMode {
    Hardware robot = Hardware.getInstance();
    public void runOpMode(){
        //once you press init

        robot.init(hardwareMap)  ;
        telemetry.addData("Status", "Hello, Drivers!");
        telemetry.update();

        //once you press play
        waitForStart();
        boolean pressingb = false;
        boolean pressingx = true;
        // boolean pressingy = false;

        boolean difference = false;
        while (opModeIsActive()){

            double leftStickY = -gamepad1.left_stick_y; // Remember, this is reversed!
            double leftStickX = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rightStickX = gamepad1.right_stick_x;

            telemetry.addData("leftStick X", leftStickX );
            telemetry.addData("leftStick Y", leftStickY );
            telemetry.addData("rightStick X", rightStickX );
            telemetry.update();

            drive( leftStickY, leftStickX, rightStickX );
            shoot();

            /*double movement = gamepad1.left_stick_y;
            double strafing = -gamepad1.left_stick_x;
            double turning = -gamepad1.right_stick_x;
            double max = Math.max(Math.abs(movement - strafing - turning),
                    Math.max(Math.abs(movement + strafing - turning),
                    Math.max(Math.abs(movement - strafing + turning),
                            Math.abs(movement + strafing + turning))));

            if (max < robot.maxSpeed){
                robot.setPower(movement - strafing - turning,
                        movement + strafing - turning,
                        movement - strafing + turning,
                        movement + strafing + turning);
            } else {
                double scaleFactor = max/robot.maxSpeed;
                robot.setPower(movement - strafing - turning * scaleFactor,
                        movement + strafing - turning * scaleFactor,
                        movement - strafing + turning * scaleFactor,
                        movement + strafing + turning * scaleFactor); }
            if(gamepad1.a){
                robot.lb.setPower(1);
            }
            if((gamepad1.left_trigger > 0.3)){
                robot.lb.setPower(0.1);
            }
            if((gamepad1.left_stick_y > 0.3)){
                robot.lb.setPower(0.1);
            }
            if(gamepad1.b && !pressingb) {
                robot.lb.setPower(0.1);
                pressingb = true;
            } else if (!gamepad1.b){
                pressingb = false;
            }
            if(gamepad1.x && !pressingx && difference) {
                robot.lb.setPower(0.1);
                pressingx = true;
                difference = false;
            } else if(gamepad1.x && !pressingx && !difference) {
                robot.rb.setPower(0.1);
                pressingx = true;
                difference = true;
            } else if (!gamepad1.x){
                pressingx = false;
            }
            // Code for shooters
//            if(gamepad1.y && !pressingy && difference){
//                robot.rs.setPower(1);
//                robot.ls.setPower(1);
//                pressingy = true;
//                difference = false;
//            } else if(gamepad1.y && !pressingy && !difference) {
//                robot.rb.setPower(0);
//                pressingx = true;
//                difference = true;
//            } else if (!gamepad1.y){
//                pressingx = false;
//            }
            if (gamepad1.dpad_left) {
                robot.demoServo.setPosition(0);
            }
            else if (gamepad1.dpad_right){
                robot.demoServo.setPosition(1);
            }
            else if (gamepad1.dpad_down){
                robot.demoServo.setPosition(0.5);
            }*/
        }
    }
    public void drive( double x, double y, double strafe ) {

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
    public void shoot(){
        // will initialize rs and ls with clicking A
        if(!gamepad1.a){
            robot.rs.setPower(0);
            robot.ls.setPower(0);
        } else{
            robot.rs.setPower(0.70);
            robot.ls.setPower(0.70);
        }
    }
}