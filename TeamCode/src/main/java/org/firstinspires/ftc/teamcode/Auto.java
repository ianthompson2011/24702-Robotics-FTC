package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//naming class so that it can be run from driverstation
@Autonomous (name = "Auto")
public class Auto extends LinearOpMode {
    Hardware robot = Hardware.getInstance();

    public void runOpMode(){
        //before auto starts
        robot.init(hardwareMap);
        //always add a telemetry.update() to make sure telemetry runs repeatedly
        telemetry.addData("status", "Hello, Drivers!");
        telemetry.update();

        //essenstially main method
        waitForStart();

        //once auto starts
        move(6, 0.67);
        move(10, 0.3);

        turning(800, 0.2);

    }
    public void move(double distance, double speed){
        //all of this depends on the type of wheel you have*
        double wheelCircumference = 4 * Math.PI;
        //revolutions per minute(rpm)
        double motar = 560;
        double ticks = (distance * (motar/wheelCircumference));

        robot.rf.setTargetPosition((int) Math.round(ticks));
        robot.lf.setTargetPosition((int)Math.round(ticks));
        robot.rb.setTargetPosition((int)Math.round(ticks));
        robot.lb.setTargetPosition((int)Math.round(ticks));

        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setPower(speed,speed,speed,speed);
        while(opModeIsActive() && robot.lb.isBusy()){

        }
        robot.setPower(0, 0, 0 , 0);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void turning(int ticks, double speed){

        robot.rf.setTargetPosition(ticks);
        robot.lf.setTargetPosition(-ticks);
        robot.rb.setTargetPosition(ticks);
        robot.lb.setTargetPosition(-ticks);

        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setPower(speed,speed,-speed,-speed);
        while(opModeIsActive() && robot.lb.isBusy()){

        }
        robot.setPower(0, 0, 0 , 0);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    }

