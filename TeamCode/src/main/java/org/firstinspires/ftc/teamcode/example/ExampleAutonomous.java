package org.firstinspires.ftc.teamcode.example;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.ExampleDriveTrain;
import org.firstinspires.ftc.teamcode.hardware.ExampleServo;
import org.firstinspires.ftc.teamcode.hardware.OpModeIsActive;

@Autonomous(name = "ExampleAutonomous", group="Example")
public class ExampleAutonomous extends LinearOpMode implements OpModeIsActive {

    static final double CLAW_OPEN = 0.714;
    static final double CLAW_CLOSED = 0.970;
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        ExampleDriveTrain driveTrain = new ExampleDriveTrain( hardwareMap, true );
        ExampleServo claw = new ExampleServo( hardwareMap, "claw");
        ExampleServo arm = new ExampleServo( hardwareMap, "arm");

        waitForStart();

        driveTrain.drive( DRIVE_SPEED, 24.0, 24.0, 10.0, this, telemetry );  // S1: Forward 24 Inches with 10 Sec timeout

        sleep(2000 );

        driveTrain.drive( TURN_SPEED,12, -12, 10.0, this, telemetry );  // S2: Turn Right 12 Inches with 10 Sec timeout

        sleep(2000 );

        driveTrain.drive( DRIVE_SPEED, -24, -24, 10.0, this, telemetry );  // S3: Reverse 24 Inches with 10 Sec timeout

        sleep(4000 );
    }

    @Override
    public boolean isActive() {
        return opModeIsActive();
    }
}
