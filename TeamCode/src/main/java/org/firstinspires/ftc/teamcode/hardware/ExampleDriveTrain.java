package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ExampleDriveTrain {

    DcMotor LB;
    DcMotor RB;
    double leftTargetDistance;
    double rightTargetDistance;
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 2.875;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public ExampleDriveTrain( HardwareMap hardwareMap, boolean useEncoders ) {

        LB = hardwareMap.dcMotor.get("LB");
        LB.setPower(0);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setDirection(DcMotor.Direction.REVERSE);

        RB = hardwareMap.dcMotor.get("RB");
        RB.setPower(0);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if( useEncoders ) {
            LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        } else {
            LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void drive( double x, double y ) {

        if( y > 0 || y < 0 ) {
            LB.setPower(0.75 * y);
            RB.setPower(0.75 * y);

        }
        else if( x > 0 ) {
            LB.setPower(0.75 * x);
            RB.setPower(0.75 * -x);

        }
        else if( x < 0 ) {
            LB.setPower(0.75 * x);
            RB.setPower(0.75 * -x);
        }
        else{
            LB.setPower(0);
            RB.setPower(0);
        }
    }

    public void drive( double speed, double leftInches, double rightInches, double timeout,
                       OpModeIsActive opMode, Telemetry telemetry ) {

        // Ensure that the opmode is still active
        if (opMode.isActive()) {

            // reset the timeout time
            runtime.reset();

            startMoving( speed, leftInches, rightInches );

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while( opMode.isActive() &&
                    runtime.seconds() < timeout &&
                    !hasReachedTarget() ) {

                displayPosition( telemetry, "Running" );
            }

            stopMoving();

            displayPosition( telemetry, "Stopped" );

            //  sleep(250);   // optional pause after each move
        }
    }

    public void startMoving( double speed, double leftInches, double rightInches ) {

        leftTargetDistance = leftInches;
        rightTargetDistance = rightInches;

        // Determine new target position, and pass to motor controller
        int newLeftTarget = LB.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        LB.setTargetPosition( newLeftTarget );

        int newRightTarget = RB.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        RB.setTargetPosition( newRightTarget );

        // Turn On RUN_TO_POSITION
        LB.setMode( DcMotor.RunMode.RUN_TO_POSITION );
        RB.setMode( DcMotor.RunMode.RUN_TO_POSITION );

        LB.setPower( Math.abs(speed) );
        RB.setPower( Math.abs(speed) );
    }

    public void stopMoving() {
        // Stop all motion;
        LB.setPower( 0 );
        RB.setPower( 0 );

        // Turn off RUN_TO_POSITION
        LB.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
        RB.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
    }

    public boolean hasReachedTarget() {

        return !(LB.isBusy() && RB.isBusy());
    }

    public void displayPosition( Telemetry telemetry, String status ) {

        telemetry.addData("Target Distance", "%4.2f\" : %4.2f\"", leftTargetDistance, rightTargetDistance );
        telemetry.addData("Delta Distance", "%4.2f\" : %4.2f\"",
                ( LB.getTargetPosition() - LB.getCurrentPosition() ) / COUNTS_PER_INCH,
                      ( RB.getTargetPosition() - RB.getCurrentPosition() ) / COUNTS_PER_INCH );
        telemetry.addData("Target", "Running to %7d : %7d", LB.getTargetPosition(), RB.getTargetPosition() );
        telemetry.addData("Position", "Running at %7d : %7d", LB.getCurrentPosition(), RB.getCurrentPosition() );
        telemetry.addData("Status", status );
        telemetry.update();
    }
}
