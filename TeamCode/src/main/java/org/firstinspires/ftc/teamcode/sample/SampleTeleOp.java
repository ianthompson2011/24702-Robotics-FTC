/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.sample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.SampleColorSensor;
import org.firstinspires.ftc.teamcode.hardware.SampleIRSensor;
import org.firstinspires.ftc.teamcode.hardware.SampleMotor;
import org.firstinspires.ftc.teamcode.hardware.SampleServo;
import org.firstinspires.ftc.teamcode.hardware.SampleTouchSensor;

/**
 * This particular OpMode demonstrates how to access a variety of motors and sensors
 */

@TeleOp(name="SampleTeleOp", group="Sample")
public class SampleTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The constructor of the hardware class does all the work here
         */
        SampleMotor motor = new SampleMotor( hardwareMap, "m0", false );
 //       SampleServo servo = new SampleServo( hardwareMap, "demo servo" );
 //       SampleTouchSensor touchSensor = new SampleTouchSensor( hardwareMap, "demo touch" );
        SampleColorSensor colorSensor = new SampleColorSensor( hardwareMap, "i0", telemetry );
        SampleIRSensor irSensor = new SampleIRSensor( hardwareMap, "i1");

        /* Declare OpMode members. */
        final double    SERVO_SPEED     = 0.02 ;                   // sets rate to move servo

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while( opModeIsActive() ) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            double direction = -gamepad1.left_stick_y;

            // Output the safe vales to the motor drives.
            motor.setPower( direction );
/*
            // Use gamepad left & right Bumpers to open and close the servo
            if (gamepad1.right_bumper)
                servo.updatePosition( SERVO_SPEED );
            else if (gamepad1.left_bumper)
                servo.updatePosition( -SERVO_SPEED );
*/
    /*
            // Use gamepad buttons to move arm up (Y) and down (A)
            if (gamepad1.y)
                robot.leftArm.setPower(robot.ARM_UP_POWER);
            else if (gamepad1.a)
                robot.leftArm.setPower(robot.ARM_DOWN_POWER);
            else
                robot.leftArm.setPower(0.0);
*/
            // Send telemetry message to signify robot running;
            telemetry.addData("direction",  "%.2f", direction );
//            telemetry.addData("servo",  "Offset = %.2f", servo.getPosition() );
//            telemetry.addData( "touch", touchSensor.isPressed() ? "true" : "false" );
            telemetry.addData( "ir detected", irSensor.isSignalDetected() ? "true" : "false" );
            if( irSensor.isSignalDetected() ) {
                telemetry.addData( "ir angle", "%.2f", irSensor.getAngle() );
                telemetry.addData( "ir strength", "%.2f", irSensor.getStrength() );
            }

            // check the status of the x button on either gamepad.
            boolean bCurrState = gamepad1.x;

            colorSensor.updateColor( bCurrState );

            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
