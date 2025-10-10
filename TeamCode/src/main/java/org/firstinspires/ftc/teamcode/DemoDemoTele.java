package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp( name = "Appears on Driver Station")
public class DemoDemoTele extends LinearOpMode {
    Hardware robot = Hardware.getInstance();

    public void runOpMode(){
        robot.init(hardwareMap);
        telemetry.addData("Status", "Hello, Drivers!");
        telemetry.update();

        waitForStart();
        boolean pressingb = false;
        boolean pressingx = false;
        boolean difference = false;
        // happens once you press play

        while(opModeIsActive()){
            double movement = gamepad1.right_stick_x;
            double strafing = -gamepad1.right_stick_y;
            double turning = -gamepad1.left_stick_y;

            double max = Math.max(Math.abs(movement - strafing - turning),
                            Math.max(Math.abs(movement + strafing - turning),
                                 Math.max(Math.abs(movement + strafing + turning),
                                    Math.abs(movement - strafing + turning))));

            if(max < robot.maxSpeed){
                robot.setPower(movement - strafing - turning,
                        movement + strafing - turning,
                        movement + strafing + turning,
                        movement - strafing + turning);
            } else{
                double scaleFactor = max / robot.maxSpeed;
                robot.setPower(movement - strafing - turning,
                        movement + strafing - turning * scaleFactor,
                        movement + strafing + turning * scaleFactor,
                        movement - strafing + turning * scaleFactor);
            }
            if(gamepad2.a){
                robot.lb.setPower(1);
            }
            if(gamepad2.left_trigger > 0.3){
                robot.lf.setPower(1);
            }
            if(gamepad2.left_stick_y > 0.3){
                robot.lf.setPower(1);
            }

            if(gamepad2.b && !pressingb){
                robot.lb.setPower(1);
                pressingb = true;
            } else if(!gamepad1.b){
                pressingb = false;
            }

            if(gamepad2.x && !pressingx){
                if(difference){
                    robot.lb.setPower(1);
                    pressingx = true;
                    difference = false;
                } else{
                    robot.rb.setPower(-1);
                    pressingx = true;
                    difference = true;
                }
            } else if(!gamepad1.x){
                pressingx = false;
            }
        }


    }
}
