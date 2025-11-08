package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auto", group = "Robot")
public class Auto extends LinearOpMode {
    Hardware robot = Hardware.getInstance();
    private ElapsedTime runtime = new ElapsedTime();

    // Constants for encoder driving
    static final double COUNTS_PER_MOTOR_REV = 537.7; // GoBilda 5202 (adjust for your motor)
    static final double DRIVE_GEAR_REDUCTION = 1.0;   // No gear reduction
    static final double WHEEL_DIAMETER_INCHES = 3.779; // GoBilda Mecanum wheels
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Set motors to RUN_USING_ENCODER for precision
//        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        runtime.reset();

        // === Autonomous Sequence Example ===
        // encoderDrive(0.25, -1, -1, 5.0);  // Move backwards
        //encoderStrafe(0.25, 1, 5.0);     // Strafe right 12 inches
        // encoderDrive(0.5, -24, -24, 5.0); // Move backward 24 inches

        // Fire shooter once for example
        moveBack();
        shootSequence();

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /** Moves robot straight (forward or backward) */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Determine target positions
        newLeftFrontTarget = robot.lf.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newRightFrontTarget = robot.rf.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        newLeftBackTarget = robot.lb.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newRightBackTarget = robot.rb.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

        robot.lf.setTargetPosition(newLeftFrontTarget);
        robot.rf.setTargetPosition(newRightFrontTarget);
        robot.lb.setTargetPosition(newLeftBackTarget);
        robot.rb.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        robot.setPower(speed, speed, speed, speed);

        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.lf.isBusy() && robot.rf.isBusy() && robot.lb.isBusy() && robot.rb.isBusy())) {

            telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    robot.lf.getCurrentPosition(),
                    robot.rf.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion
        robot.setPower(0, 0, 0, 0);

        // Reset mode
        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** Strafes robot left/right (positive = right, negative = left) */
    public void encoderStrafe(double speed, double inches, double timeoutS) {
        int newLeftFrontTarget = robot.lf.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newRightFrontTarget = robot.rf.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        int newLeftBackTarget = robot.lb.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        int newRightBackTarget = robot.rb.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

        robot.lf.setTargetPosition(newLeftFrontTarget);
        robot.rf.setTargetPosition(newRightFrontTarget);
        robot.lb.setTargetPosition(newLeftBackTarget);
        robot.rb.setTargetPosition(newRightBackTarget);

        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        robot.setPower(speed, speed, speed, speed);

        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.lf.isBusy() && robot.rf.isBusy() && robot.lb.isBusy() && robot.rb.isBusy())) {
            telemetry.addData("Strafing", "Running");
            telemetry.update();
        }

        robot.setPower(0, 0, 0, 0);
        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** Example shoot sequence (1 shot) */

    public void moveBack() {
        robot.setPower(1, -1, 1, -1); // fr, br, bl, fl
        sleep(200);
        robot.setPower(0, 0, 0, 0);
    }

    public void shootSequence() {
        robot.rs.setPower(1);
        robot.ls.setPower(1);
        sleep(1000); // spin-up time
        robot.demoServo1.setPosition(0.25);
        robot.demoServo2.setPosition(0.25);
        sleep(3000);
        robot.demoServo1.setPosition(0.5);
        robot.demoServo2.setPosition(0.5);
        robot.rs.setPower(0);
        robot.ls.setPower(0);
    }
}
