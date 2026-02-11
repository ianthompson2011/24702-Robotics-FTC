package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

// use this auto if on the left side
// will move the robot forward and rotate slightly, then shoot three balls

@Autonomous(name = "FarGoalAutoLeft", group = "Robot")
public class FarGoalAutoLeft extends LinearOpMode {
    Hardware robot = Hardware.getInstance();

    private static final double HIGH_VELOCITY = 1500;
    private static final double LOW_VELOCITY  = 1400;
    private static final double READY_PERCENT = 0.95;
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

        move();
        shootSequence();

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void move() {
        robot.setPower(-1, 1, -1, 1); // fr, br, bl, fl
        sleep(1500); // WHEN AT 100% BATTERY
        robot.setPower(0, 0, 0, 0);
        sleep(500);
        robot.setPower(1, 1, -1, -1);
        sleep(250);
        robot.setPower(0,0,0,0);
    }

    private void shootSequence() {

        double targetVelocity = HIGH_VELOCITY;

        // Spin up shooters
        robot.rs.setVelocity(-targetVelocity);
        robot.ls.setVelocity(-targetVelocity);

        // Wait until shooters are ready (or timeout safety)
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive()
                && timer.seconds() < 30
                && !(Math.abs(robot.rs.getVelocity()) > targetVelocity * READY_PERCENT
                && Math.abs(robot.ls.getVelocity()) > targetVelocity * READY_PERCENT)) {
            idle();
        }

        // Fire preloaded balls
        robot.it.setPower(1);
        robot.demoServo1.setPosition(0.75);
        sleep(5000);

        // Reset
        robot.it.setPower(0);
        robot.demoServo1.setPosition(0.5);
        sleep(200);

        // Stop shooters
        robot.rs.setVelocity(0);
        robot.ls.setVelocity(0);
    }
}