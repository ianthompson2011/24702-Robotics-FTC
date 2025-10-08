package org.firstinspires.ftc.teamcode.minigame;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.MinigameGamepad;
import org.firstinspires.ftc.teamcode.hardware.MinigameMotor;
import org.firstinspires.ftc.teamcode.hardware.MinigameServo;
import org.firstinspires.ftc.teamcode.hardware.MinigameColorSensor;
import org.firstinspires.ftc.teamcode.hardware.MinigameColorSensor.StripColor;
import org.firstinspires.ftc.teamcode.hardware.MinigameIRSensor;
import org.firstinspires.ftc.teamcode.hardware.OpModeIsActive;

/**
 * Device  Port   Variable name               Control Filename
 * Motor   Port 0 leftMotor                        m0
 * Motor   Port 1 rightMotor                       m1
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

@Autonomous(name = "MinigameAutonomous", group="Minigame")
public class MinigameAutonomous extends LinearOpMode implements OpModeIsActive {

    enum State { WaitingForStart, StartMoving, LookingForColor, LookingForIRSensor, StartDroppingCube, FinishedDroppingCube, MovingToEnd, Turning, MovingToFinalPosition, Stopped }

    protected StripColor sideColor = StripColor.NoColor;

    static final double BASKET_INIT = 0.655;
    static final double BASKET_UP = 0.542;
    static final double BASKET_DOWN = 0.493;

    static final double BLOCK_INIT = 0.915;
    static final double BLOCK_DROP = 0.25;

    static final double DRIVE_SPEED = 0.50;
    static final double TURN_SPEED = 0.50;

    static final double SCOOP_SPEED = 0.3;
    static final int SCOOP_UP = 1000;

    static final double DRIVE_FUDGE_FACTOR = 1.1;

    static final double ARC_RADIUS = 41;

    static final double DECISION_DISTANCE = ARC_RADIUS + 2;

    static final double DISTANCE_ACROSS_FIELD = 140 - DECISION_DISTANCE - 14;

    static final double DISTANCE_TO_FINAL_POSITION = DECISION_DISTANCE - 9;
    static final double DISTANCE_TO_MIDDLE_STRIP = 4;

    public MinigameAutonomous() {
        telemetry.addData( "Creating MinigameAutonomous", sideColor );
        telemetry.update();

    }
    @Override
    public void runOpMode() {

        telemetry.addData( "Initializing hardware", sideColor );
        telemetry.update();

        MinigameIRSensor.MinigameDriveTrain driveTrain = new MinigameIRSensor.MinigameDriveTrain( hardwareMap, "m0", "m1", true );
        MinigameMotor scoop = new MinigameMotor( hardwareMap, "m3", false );
        MinigameColorSensor colorSensor = new MinigameColorSensor( hardwareMap, "i0" );
        MinigameIRSensor irSensor = new MinigameIRSensor( hardwareMap, "i1" );
        MinigameServo block = new MinigameServo( hardwareMap, "s0");
        MinigameServo basket = new MinigameServo( hardwareMap, "s1");
        MinigameGamepad myGamepad = new MinigameGamepad( gamepad1 );

        basket.setPosition( BASKET_INIT );
        block.setPosition( BLOCK_INIT );

        while( sideColor == StripColor.NoColor ) {

            if( myGamepad.isXPressed() ) {
                sideColor = StripColor.Blue;
            } else if( myGamepad.isBPressed() ) {
                sideColor = StripColor.Red;
            }

            if( myGamepad.getLeftTrigger() != 0 ) {
                scoop.setPower( -SCOOP_SPEED );
            } else if( myGamepad.getRightTrigger() != 0 ) {
                scoop.setPower( SCOOP_SPEED );
            } else {
                scoop.setPower( 0.0 );
            }

            telemetry.addData( "scoop encoder", scoop.getEncoderPosition() );
            telemetry.update();
        }

        telemetry.addData( "sideColor", sideColor );
        telemetry.update();

        scoop.changeToUseEncoders();

        waitForStart();

        scoop.startMoving( SCOOP_SPEED, SCOOP_UP );

        State state = State.WaitingForStart;
        StripColor stripColor = StripColor.NoColor;
        long sleepTime = 0;

        double leftDriveSpeed = DRIVE_SPEED;
        double rightDriveSpeed = DRIVE_SPEED;

        if( sideColor == StripColor.Blue ) {
            rightDriveSpeed *= DRIVE_FUDGE_FACTOR;
        }

        while( opModeIsActive() ) {

            telemetry.addData( "State", state );
            telemetry.addData( "IR Signal Detected", irSensor.isSignalDetected() );
            telemetry.addData( "StripColor", stripColor );

            switch( state ) {

                case WaitingForStart:

                   state = State.StartMoving;

                    break;

                case StartMoving:

                    double leftDistance = DISTANCE_ACROSS_FIELD;
                    double rightDistance = DISTANCE_ACROSS_FIELD;

                    if( sideColor == StripColor.Blue ) {
                        rightDistance *= DRIVE_FUDGE_FACTOR;
                    } else {
                        leftDistance = -leftDistance;
                        rightDistance = -rightDistance;
                    }

                    driveTrain.startMoving( leftDriveSpeed, rightDriveSpeed, leftDistance, rightDistance );

                    state = State.LookingForColor;

                    break;

                case LookingForColor:

                    stripColor = colorSensor.getColor();

                    if( stripColor != StripColor.NoColor ) {

                        state = State.LookingForIRSensor;
                    }

                    break;

                case LookingForIRSensor:

                    if( irSensor.isSignalDetected() ) {

                        telemetry.addData( "IR Signal Angle", irSensor.getAngle() );
                        telemetry.addData( "IR Signal Strength", irSensor.getStrength() );

                        driveTrain.pauseMoving();

                        state = State.StartDroppingCube;
                    }

                    break;

                case StartDroppingCube:

                    block.setPosition( BLOCK_DROP );

                    sleepTime = 2000;

                    state = State.FinishedDroppingCube;

                    break;

                case FinishedDroppingCube:

                    driveTrain.resumeMoving( leftDriveSpeed, rightDriveSpeed );
                    
                    state = State.MovingToEnd;

                    break;

                case MovingToEnd:

                    if( driveTrain.hasReachedTarget() ) {

                        driveTrain.stopMoving();

                        if( stripColor != sideColor ) {
                            state = State.Turning;
                        } else {
                            state = State.MovingToFinalPosition;
                        }
                    }
                    
                    break;

                case Turning:

                    if( sideColor == StripColor.Blue ) {
                        driveTrain.driveForwardViaArc(-90, TURN_SPEED, ARC_RADIUS, 20, this, telemetry );
                    } else {
                        driveTrain.driveBackwardViaArc(-90, TURN_SPEED, ARC_RADIUS, 20, this, telemetry );
                    }

                    state = State.MovingToFinalPosition;

                    break;

                case MovingToFinalPosition:

                    if( sideColor == StripColor.Blue ) {
                        if( stripColor == StripColor.Blue ) {
                            leftDistance = DISTANCE_TO_FINAL_POSITION;
                            rightDistance = DISTANCE_TO_FINAL_POSITION * DRIVE_FUDGE_FACTOR;
                        } else {
                            leftDistance = DISTANCE_TO_MIDDLE_STRIP;
                            rightDistance = DISTANCE_TO_MIDDLE_STRIP * DRIVE_FUDGE_FACTOR;
                        }
                    } else {
                        if( stripColor == StripColor.Red ) {
                            leftDistance = -DISTANCE_TO_FINAL_POSITION;
                            rightDistance = -DISTANCE_TO_FINAL_POSITION;
                        } else {
                            leftDistance = -(DISTANCE_TO_MIDDLE_STRIP + 12);
                            rightDistance = -(DISTANCE_TO_MIDDLE_STRIP + 12);
                        }
                    }

                    driveTrain.driveForward( leftDriveSpeed, rightDriveSpeed, leftDistance, rightDistance, 5, this, telemetry );

                    block.setPosition( BLOCK_INIT );

                    state = State.Stopped;

                    break;

                case Stopped:

                    break;
            }

            driveTrain.displayPosition( telemetry, 30, "Moving" );

            if( sleepTime > 0 ) {
                sleep( sleepTime );
                sleepTime = 0;
            }
        }

        driveTrain.stopMoving();

        driveTrain.displayPosition( telemetry, 30, "Stopped" );
    }

    @Override
    public boolean isActive() {
        return opModeIsActive();
    }
}
