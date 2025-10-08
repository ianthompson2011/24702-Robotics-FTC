package org.firstinspires.ftc.teamcode.minigame;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.MinigameGamepad;
import org.firstinspires.ftc.teamcode.hardware.MinigameIRSensor;
import org.firstinspires.ftc.teamcode.hardware.MinigameMotor;
import org.firstinspires.ftc.teamcode.hardware.MinigameServo;
import org.firstinspires.ftc.teamcode.hardware.OpModeIsActive;

/**
 * Device  Port   Variable name               Control Filename
 * Motor   Port 0 leftMotor                        m0
 * Motor   Port 1 rightMotor                       m1
 * Motor   Port 3 basket                           m3
 *
 * Servo   Port 0 block                            s0
 * Servo   Port 1 basket                           s1
 * Servo   Port 2                                  s2
 * Servo   Port 3                                  s3
 * Servo   Port 4                                  s4
 * Servo   Port 5                                  s5
 *
 * I2C     Port 0 colorSensor                      i0
 * I2C     Port 1 IRSensor                         i1
 * I2C     Port 2                                  i2
 * I2C     Port 3                                  i3
 */
@TeleOp(name = "MinigameTeleOp", group="Minigame")
public class MinigameTeleOp extends LinearOpMode implements OpModeIsActive {

    enum DriveType {

        TankDrive, ArcadeDriveLeftJoystick, ArcadeDriveRightJoystick, Variant;

        private static final DriveType[] vals = values();

        public DriveType next() {
            return vals[(this.ordinal() + 1) % vals.length];
        }
    }

    static final double BASKET_INIT = 0.655;
    static final double BASKET_UP = 0.600;

    static final double BASKET_DRIVE = 0.550;

    static final double BASKET_DOWN = 0.493;

    static final double BLOCK_INIT = 0.435;

    static final double SCOOP_SPEED = 0.3;
    static final int SCOOP_DOWN = 40;
    static final int SCOOP_UP = 2400;

    public void runOpMode() {

        MinigameIRSensor.MinigameDriveTrain driveTrain = new MinigameIRSensor.MinigameDriveTrain( hardwareMap, "m0", "m1", false );
        MinigameMotor scoop = new MinigameMotor( hardwareMap, "m3", true );
        MinigameServo block = new MinigameServo( hardwareMap, "s0" );
        MinigameServo basket = new MinigameServo( hardwareMap, "s1" );
        MinigameGamepad myGamepad1 = new MinigameGamepad( gamepad1 );
        MinigameGamepad myGamepad2 = new MinigameGamepad( gamepad2 );

        // Define some variables
        double leftPower = 0.0;
        double rightPower = 0.0;
        double speedFactor = 0.5;
        DriveType driveType = DriveType.TankDrive;
        boolean gamepadPressed = false;
        boolean scoopUp = false;

        basket.setPosition( BASKET_INIT );
        block.setPosition( BLOCK_INIT );

        while( !myGamepad2.isXPressed() ) {
            if( myGamepad2.getLeftTrigger() != 0 ) {
                scoop.setPower( -SCOOP_SPEED );
            } else if( myGamepad2.getRightTrigger() != 0 ) {
                scoop.setPower( SCOOP_SPEED );
            } else {
                scoop.setPower( 0.0 );
            }
        }

        telemetry.addLine( "WaitingForStart" );
        telemetry.update();

        scoop.changeToUseEncoders();

        scoop.startMoving( SCOOP_SPEED, SCOOP_DOWN );

        waitForStart();

        basket.setPosition( BASKET_DOWN );
        
        while( opModeIsActive() ) {

            double leftStickY = -myGamepad1.getLeftStickY();
            double leftStickX = myGamepad1.getLeftStickX();

            double rightStickY = -myGamepad1.getRightStickY();
            double rightStickX = myGamepad1.getRightStickX();

            // Select type of drive controls
            if( myGamepad1.isXPressed() ) {
                driveType = driveType.next();
            }

            // Change speed factor
            if (myGamepad1.isYPressed() ) {
                if( speedFactor < 1.0 ) {
                    speedFactor += 0.25;
                }
            }

            if( myGamepad1.isAPressed() ) {
                if( speedFactor > 0.5 ) {
                    speedFactor -= 0.25;
                }
            }

            if( myGamepad2.isLeftBumperPressed() ) {
                if( basket.getPosition() != BASKET_DOWN ) {
                    basket.setPosition( BASKET_DOWN );
                }
            }

            if( myGamepad2.isRightBumperPressed() ) {
                if( basket.getPosition() != BASKET_UP ) {
                    basket.setPosition( BASKET_UP );
                }
            }

            if( myGamepad2.isBPressed() ) {
                if( scoopUp ) {
                    scoopUp = false;
                    scoop.startMoving( SCOOP_SPEED, SCOOP_DOWN );
                    basket.setPosition( BASKET_DRIVE );
                } else {
                    scoopUp = true;
                    scoop.startMoving( SCOOP_SPEED, SCOOP_UP );
                    basket.setPosition( BASKET_UP );
                }
            }

            //Slow speed - Uses the dpad instead of the joystick
            if( myGamepad1.isDpadLeftPressed() || myGamepad1.isDpadRightPressed() || myGamepad1.isDpadUpPressed() || myGamepad1.isDpadDownPressed() ) {

                gamepadPressed = true;

                if (myGamepad1.isDpadLeftPressed()) {           // Slow Turn Left
                    leftPower = -0.2;
                    rightPower = 0.2;

                } else if (myGamepad1.isDpadRightPressed()) {   // Slow Turn Right
                    leftPower = 0.2;
                    rightPower = -0.2;

                } else if (myGamepad1.isDpadUpPressed()) {      // Slow Forward
                    leftPower = 0.2;
                    rightPower = 0.2;

                } else if (myGamepad1.isDpadDownPressed()) {     // Slow Backward
                    leftPower = -0.2;
                    rightPower = -0.2;

                }
            } else if( gamepadPressed ) {

                leftPower = 0.0;
                rightPower = 0.0;

            } else {

                switch( driveType ) {

                    case TankDrive:

                        //Tank Drive
                        telemetry.addData("Drive Type: ", "Tank" );
                        telemetry.addData("leftStick Y: ", leftStickY );
                        telemetry.addData("rightStick Y: ", rightStickY );
                        telemetry.addData("speedFactor: ", speedFactor );

                        leftPower = leftStickY * speedFactor;
                        rightPower = rightStickY * speedFactor;

                        break;

                    case ArcadeDriveLeftJoystick:

                        //Arcade Drive left joystick
                        telemetry.addData("Drive Type: ", "Arcade (Left Stick)" );
                        telemetry.addData("leftStick Y: ", leftStickY );
                        telemetry.addData("leftStick X: ", leftStickX );
                        telemetry.addData("speedFactor: ", speedFactor );

                        leftPower = leftStickY * speedFactor + leftStickX * .5 * speedFactor;
                        rightPower = leftStickY * speedFactor - leftStickX * .5 * speedFactor;

                        break;

                    case ArcadeDriveRightJoystick:

                        //Arcade Drive Right joystick
                        telemetry.addData("Drive Type: ", "Arcade (Right Stick)" );
                        telemetry.addData("rightStick Y: ", rightStickY );
                        telemetry.addData("rightStick X: ", rightStickX );
                        telemetry.addData("speedFactor: ", speedFactor );

                        leftPower = rightStickY * speedFactor + rightStickX * .5 * speedFactor;
                        rightPower = rightStickY * speedFactor - rightStickX * .5 * speedFactor;

                        break;

                    default:

                        //Variant
                        telemetry.addData("Drive Type: ", "Variant (Left-Power  Right-Turns");
                        telemetry.addData("leftStick Y: ", leftStickY );
                        telemetry.addData("rightStick X: ", rightStickX );
                        telemetry.addData("speedFactor: ", speedFactor );

                        leftPower = leftStickY * speedFactor + rightStickX * .5 * speedFactor;
                        rightPower = leftStickY * speedFactor - rightStickX * .5 * speedFactor;

                        break;
                }
            }

            telemetry.addData("leftPower: ", leftPower );
            telemetry.addData("rightPower: ", rightPower );
            telemetry.update();

            driveTrain.drive( leftPower, rightPower );
        }
    }

    @Override
    public boolean isActive() {
        return opModeIsActive();
    }
}
