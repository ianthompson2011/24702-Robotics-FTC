package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//time based: uses time, does not account for slippage
//encoder: counts ticks to get to position, slippage
//odometry: corrects position

//naming class so that it can be run from driver station
@Autonomous (name = "DemoAuto")
public class DemoAuto extends LinearOpMode {
    Hardware robot = Hardware.getInstance();

    //runOpMode() are like a main method
    public void runOpMode() {
        //code here runs before auto starts (once int is pressed on driver station)
        robot.init(hardwareMap);

        //always add telemetry.update() to make sure telemetry runs repeatedly
        telemetry.addData("Status", "Hello, Drivers!");
        telemetry.update();

        waitForStart();
        //code here runs once auto starts (once play is pressed on driver station)

        //calling on move method to move forward 6in at 67% speed
        move(6, 0.67);
        //calling on move method to move forward 10in at 30% speed
        move(10, 0.3);
        //calling on turning method to turn 800 ticks at 20% speed
        turning(800, 0.2);
    }
    public void move(double distance, double speed) {

        //converting ticks to inches - change to match the wheels and motors you are using
        double wheelCirumference = 4 * Math.PI;
        double motor = 560;
        double ticks = (distance * (motor/wheelCirumference));

        //type casting ticks to be an integer to match required varible type
        robot.rf.setTargetPosition((int) Math.round(ticks));
        robot.lf.setTargetPosition((int) Math.round(ticks));
        robot.rb.setTargetPosition((int) Math.round(ticks));
        robot.lb.setTargetPosition((int) Math.round(ticks));

        //resetting encoders so that previously stored values are not used
        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set run to position
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set speed
        robot.setPower(speed, speed, speed, speed);

        //above code will run until the condition is satisfied
        while (opModeIsActive() && robot.lb.isBusy()) {

        }

        //power set to 0, without this the previous code will continue rather than stop
        robot.setPower(0,0,0,0);

        //set back to run with encoders only
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void turning(int ticks, double speed) {

        //using ticks to turn
        robot.rf.setTargetPosition(ticks);
        robot.lf.setTargetPosition(-ticks);
        robot.rb.setTargetPosition(ticks);
        robot.lb.setTargetPosition(-ticks);

        //resetting encoders so that previously stored values are not used
        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set run to position
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set speed, reversing two motors from the same side of the robot so one side turns one direction and the other the opposite
        robot.setPower(speed, speed, -speed, -speed);
        while (opModeIsActive() && robot.lb.isBusy()) {

        }
        robot.setPower(0,0,0,0);

        //switching back to just running with encoders
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}