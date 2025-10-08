package org.firstinspires.ftc.teamcode.helloworld;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode includes all the skeletal structure that all linear OpModes contain.
 */

@TeleOp(name="Hello World", group="Example")
public class HelloWorld extends LinearOpMode {

    // Declare OpMode members.
    @Override
    public void runOpMode() {

        telemetry.addData("Say", "Hello Driver" );
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Start", "Hello World!" );
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while( opModeIsActive() ) {

        }
    }
}
