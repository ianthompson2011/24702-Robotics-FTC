package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

// The purpose of this auto is to move to receive points for auto
// Only use if other team's auto is better than ours
// Used to avoid interference with other team's auto while still getting points

@Autonomous(name = "DoNothingAuto", group = "Robot")
public class DoNothingAuto extends LinearOpMode {
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

        telemetry.update();
    }

    public void moveBack() {
        robot.setPower(-1, 1, -1, 1); // fr, br, bl, fl
        sleep(200); // WHEN AT 100% BATTERY
        robot.setPower(0, 0, 0, 0);
    }
}