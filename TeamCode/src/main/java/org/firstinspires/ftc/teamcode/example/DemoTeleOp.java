package org.firstinspires.ftc.teamcode.example;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DemoDriveTrain;
import org.firstinspires.ftc.teamcode.hardware.ExampleServo;

@TeleOp(name = "DemoTeleOp", group="Example")
public class DemoTeleOp extends LinearOpMode {

    static final double CLAW_OPEN = 0.714;
    static final double CLAW_CLOSED = 0.970;

    public void runOpMode() {

        DemoDriveTrain driveTrain = new DemoDriveTrain( hardwareMap, false );
 //       ExampleServo claw = new ExampleServo( hardwareMap, "claw");
 //       ExampleServo arm = new ExampleServo( hardwareMap, "arm");

        waitForStart();

        while( opModeIsActive() && !gamepad2.a ) {

            double leftStickY = -gamepad1.left_stick_y; // Remember, this is reversed!
            double leftStickX = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rightStickX = gamepad1.right_stick_x;

            telemetry.addData("leftStick X", leftStickX );
            telemetry.addData("leftStick Y", leftStickY );
            telemetry.addData("rightStick X", rightStickX );
            telemetry.update();

            driveTrain.drive( leftStickX, leftStickY, rightStickX );
/*
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
 */
        }
    }
}
