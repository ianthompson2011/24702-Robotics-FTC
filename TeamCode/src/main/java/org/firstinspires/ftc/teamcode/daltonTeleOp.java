package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Appears on Driver Station")

public class daltonTeleOp extends LinearOpMode {
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
            double movement = gamepad1.left_stick_x;
            double strafing = -gamepad1.right_stick_y;
            double turning = -gamepad1.left_stick_y;

            double max = Math.max(Math.abs(movement - strafing - turning),
                    Math.max(Math.abs(movement + strafing - turning),
                    Math.max(Math.abs(movement + strafing + turning),
                            Math.abs(movement - strafing + turning))));

            if (max < robot.maxSpeed){
                robot.setPower(movement - strafing - turning,
                        movement + strafing - turning,
                        movement + strafing + turning,
                        movement - strafing + turning);
            } else {
                double scaleFactor = max/robot.maxSpeed;
                robot.setPower(movement - strafing - turning * scaleFactor,
                        movement + strafing - turning * scaleFactor,
                        movement + strafing + turning * scaleFactor,
                        movement - strafing + turning * scaleFactor); }
            if(gamepad1.a){
                robot.lb.setPower(1);
            }
            if((gamepad1.left_trigger > 0.3)){
                robot.lb.setPower(1);
            }
            if((gamepad1.left_stick_y > 0.3)){
                robot.lb.setPower(1);
            }
            if(gamepad1.b && !pressingb) {
                robot.lb.setPower(1);
                pressingb = true;
            } else if (!gamepad1.b){
                pressingb = false;
            }
            if(gamepad1.x && !pressingx && difference) {
                robot.lb.setPower(1);
                pressingx = true;
                difference = false;
            } else if(gamepad1.x && !pressingx && !difference) {
                robot.rb.setPower(-1);
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
            }
        }
    }

}
