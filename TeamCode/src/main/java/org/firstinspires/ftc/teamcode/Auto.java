package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//naming class so that it can be run from driverstation
@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    Hardware robot = Hardware.getInstance();

    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Ready to start");
        telemetry.update();

        waitForStart();

        // Move forward to shooting position
        move(10, 0.4);

        // Spin up shooters
        robot.ls.setPower(1);
        robot.rs.setPower(1);
        sleep(1500);

        // Feed 2 balls
        for (int i = 0; i < 2; i++) {
            robot.demoServo1.setPosition(1);
            sleep(500);
            robot.demoServo2.setPosition(0);
            sleep(700);
        }

        // Stop shooter
        robot.ls.setPower(0);
        robot.rs.setPower(0);

        // Optional: back away or park
        move(-5, 0.4);
    }

    public void move(double distance, double speed) {
        double wheelCircumference = 4 * Math.PI;
        double ticksPerRev = 560;
        double ticks = (distance * (ticksPerRev / wheelCircumference));

        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rf.setTargetPosition((int) Math.round(ticks));
        robot.lf.setTargetPosition((int) Math.round(ticks));
        robot.rb.setTargetPosition((int) Math.round(ticks));
        robot.lb.setTargetPosition((int) Math.round(ticks));

        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setPower(speed, speed, speed, speed);
        while (opModeIsActive() && robot.lb.isBusy()) {
            telemetry.addData("Moving", "Running to %.0f ticks", ticks);
            telemetry.update();
        }

        robot.setPower(0, 0, 0, 0);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
