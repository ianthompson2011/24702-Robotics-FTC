package org.firstinspires.ftc.teamcode.example;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.ExampleDriveTrain;
import org.firstinspires.ftc.teamcode.hardware.ExampleServo;

@TeleOp(name = "ExampleTeleOp", group="Example")
public class ExampleTeleOp extends LinearOpMode {

    static final double CLAW_OPEN = 0.714;
    static final double CLAW_CLOSED = 0.970;

    public void runOpMode() {

        ExampleDriveTrain driveTrain = new ExampleDriveTrain( hardwareMap, false );
        ExampleServo claw = new ExampleServo( hardwareMap, "claw");
        ExampleServo arm = new ExampleServo( hardwareMap, "arm");

        waitForStart();

        while( opModeIsActive() && !gamepad2.a ) {

            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = gamepad1.left_stick_y * -1;

            telemetry.addData("leftStick X", leftStickX );
            telemetry.addData("leftStick Y", leftStickY );
            telemetry.update();

            driveTrain.drive( leftStickX, leftStickY );

            if( gamepad1.x ) {
                claw.setPosition( CLAW_OPEN );

            } else if( gamepad1.y ) {
                claw.setPosition( CLAW_CLOSED );
            }

            if( gamepad1.a ) {
                arm.setPosition( 0.721 );

            } else if( gamepad1.b ) {
                arm.setPosition( 0.853 );

            } else if( gamepad1.right_bumper ) {
                arm.setPosition( 0.650 );
            }
        }
    }
}
