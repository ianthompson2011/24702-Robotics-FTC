package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

// move backwards and shoot 3 balls

@Autonomous(name = "NearGoalAuto", group = "Robot")
public class NearGoalAuto extends LinearOpMode {
    Hardware robot = Hardware.getInstance();
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        runtime.reset();

        moveBack();
        shootSequence();

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void moveBack() {
        robot.setPower(1, -1, 1, -1); // fr, br, bl, fl
        sleep(650); // WHEN AT 100% BATTERY
        robot.setPower(0, 0, 0, 0);
    }

    public void shootSequence(){
        robot.rs.setPower(-0.5);
        robot.ls.setPower(-0.5);
        sleep(1250);
        robot.it.setPower(1);
        sleep(1500);
        robot.demoServo1.setPosition(0.75);
        sleep(1750);
        robot.demoServo1.setPosition(0.25);
        sleep(500);
        robot.demoServo1.setPosition(0.5);
    }
}