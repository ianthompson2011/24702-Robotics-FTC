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

package org.firstinspires.ftc.teamcode.hardware;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MinigameIRSensor {

  IrSeekerSensor irSeeker;    // Hardware Device Object

  static final double SIGNAL_THRESHOLD = 0.50;

  public MinigameIRSensor( HardwareMap hardwareMap, String deviceName ) {

    // get a reference to our GyroSensor object.
    irSeeker = hardwareMap.irSeekerSensor.get( deviceName );
  }

  public boolean isSignalDetected() {

    return irSeeker.signalDetected() && irSeeker.getStrength() > SIGNAL_THRESHOLD;
  }

  public double getAngle() {
    return irSeeker.getAngle();
  }

  public double getStrength() {
    return irSeeker.getStrength();
  }

    public static class MinigameDriveTrain {

        DcMotor leftMotor;
        DcMotor rightMotor;
        double leftTargetDistance;
        double rightTargetDistance;
        private final ElapsedTime runtime = new ElapsedTime();

        static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
        static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
        static final double     WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
        static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        static final double     DISTANCE_BETWEEN_WHEELS = 16.0;

        public MinigameDriveTrain( HardwareMap hardwareMap, String leftMotorName, String rightMotorName, boolean useEncoders ) {

            leftMotor = hardwareMap.dcMotor.get( leftMotorName );
            leftMotor.setPower(0);
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftMotor.setDirection(DcMotor.Direction.REVERSE);

            rightMotor = hardwareMap.dcMotor.get( rightMotorName );
            rightMotor.setPower(0);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if( useEncoders ) {
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            } else {
                leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

        public void drive( double leftPower, double rightPower ) {

            if( Math.abs( leftPower ) > .1 ) {
                leftMotor.setPower( leftPower );
            } else {
                leftMotor.setPower( 0 );
            }

            if( Math.abs( rightPower ) > .1 ) {
                rightMotor.setPower( rightPower );
            } else {
                rightMotor.setPower( 0 );
            }
        }

        public void driveForward( double leftSpeed, double rightSpeed, double leftInches, double rightInches, double timeout,
                                  OpModeIsActive opMode, Telemetry telemetry ) {

            drive( leftSpeed, rightSpeed, leftInches, rightInches, timeout, opMode, telemetry );
        }

        public void driveForwardViaArc( double degrees, double outsideSpeed, double outsideRadius, double timeout, OpModeIsActive opMode, Telemetry telemetry ) {

            double outsideDistance = ( 2 * PI * outsideRadius ) * ( Math.abs( degrees ) / 360.0 );
            double insideDistance = ( 2 * PI * ( outsideRadius - DISTANCE_BETWEEN_WHEELS ) ) * ( Math.abs( degrees ) / 360.0 );

            double insideSpeed = outsideSpeed * ( insideDistance / outsideDistance );

            double leftDistance = outsideDistance;
            double leftSpeed = outsideSpeed;
            double rightDistance = insideDistance;
            double rightSpeed = insideSpeed;

            if( degrees < 0 ) {
                leftDistance = insideDistance;
                leftSpeed = insideSpeed;
                rightDistance = outsideDistance;
                rightSpeed = outsideSpeed;
            }

            drive( leftSpeed, rightSpeed, leftDistance, rightDistance, timeout, opMode, telemetry );
        }

        public void driveBackwardViaArc( double degrees, double outsideSpeed, double outsideRadius, double timeout, OpModeIsActive opMode, Telemetry telemetry ) {

            double outsideDistance = ( 2 * PI * outsideRadius ) * ( Math.abs( degrees ) / 360.0 );
            double insideDistance = ( 2 * PI * ( outsideRadius - DISTANCE_BETWEEN_WHEELS ) ) * ( Math.abs( degrees ) / 360.0 );

            double insideSpeed = outsideSpeed * ( insideDistance / outsideDistance );

            double leftDistance = outsideDistance;
            double leftSpeed = outsideSpeed;
            double rightDistance = insideDistance;
            double rightSpeed = insideSpeed;

            if( degrees < 0 ) {
                leftDistance = insideDistance;
                leftSpeed = insideSpeed;
                rightDistance = outsideDistance;
                rightSpeed = outsideSpeed;
            }

            drive( leftSpeed, rightSpeed, -leftDistance, -rightDistance, timeout, opMode, telemetry );
        }

        public void turnLeft( double speed, double timeout, OpModeIsActive opMode, Telemetry telemetry ){

            drive( speed, speed, -5, 5, timeout, opMode, telemetry );
        }

        public void drive( double leftSpeed, double rightSpeed, double leftInches, double rightInches, double timeout,
                           OpModeIsActive opMode, Telemetry telemetry ) {

            // Ensure that the opmode is still active
            if (opMode.isActive()) {

                // reset the timeout time
                runtime.reset();

                startMoving( leftSpeed, rightSpeed, leftInches, rightInches );

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while( opMode.isActive() &&
                        runtime.seconds() < timeout &&
                        !hasReachedTarget() ) {

                    displayPosition( telemetry, timeout, "Running" );
                }

                stopMoving();

                displayPosition( telemetry, timeout, "Stopped" );

                //  sleep(250);   // optional pause after each move
            }
        }

        public void startMoving( double leftSpeed, double rightSpeed, double leftInches, double rightInches ) {

            leftTargetDistance = leftInches;
            rightTargetDistance = rightInches;

            // Determine new target position, and pass to motor controller
            int newLeftTarget = leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition( newLeftTarget );

            int newRightTarget = rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            rightMotor.setTargetPosition( newRightTarget );

            // Turn On RUN_TO_POSITION
            leftMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
            rightMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );

            leftMotor.setPower( Math.abs(leftSpeed) );
            rightMotor.setPower( Math.abs(rightSpeed) );
        }

        public void stopMoving() {
            // Stop all motion;
            leftMotor.setPower( 0 );
            rightMotor.setPower( 0 );

            // Turn off RUN_TO_POSITION
            leftMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
            rightMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
        }

        public void pauseMoving() {
            // Stop all motion;
            leftMotor.setPower( 0 );
            rightMotor.setPower( 0 );
        }

        public void resumeMoving( double leftSpeed, double rightSpeed ) {

            leftMotor.setPower( Math.abs( leftSpeed ) );
            rightMotor.setPower( Math.abs( rightSpeed ) );
        }

        public boolean hasReachedTarget() {

            return !(leftMotor.isBusy() && rightMotor.isBusy());
        }

        public double getPosition() {
            return leftMotor.getCurrentPosition() / COUNTS_PER_INCH;
        }

        public void displayPosition( Telemetry telemetry, double timeout, String status ) {

            telemetry.addData("Target Distance", "%4.2f\" : %4.2f\"", leftTargetDistance, rightTargetDistance );
            telemetry.addData("Delta Distance", "%4.2f\" : %4.2f\"",
                    ( leftMotor.getTargetPosition() - leftMotor.getCurrentPosition() ) / COUNTS_PER_INCH,
                          ( rightMotor.getTargetPosition() - rightMotor.getCurrentPosition() ) / COUNTS_PER_INCH );
            telemetry.addData("Target", "Running to %7d : %7d", leftMotor.getTargetPosition(), rightMotor.getTargetPosition() );
            telemetry.addData("Position", "Running at %7d : %7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition() );
            telemetry.addData( "Time Left", "%4.2f", timeout - runtime.seconds() );
            telemetry.addData("Status", status );
            telemetry.update();
        }
    }
}
