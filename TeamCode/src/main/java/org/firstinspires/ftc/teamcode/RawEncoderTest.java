package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "RawEncoderTest")
public class RawEncoderTest extends LinearOpMode {

    private DcMotor forwardPod;
    private DcMotor strafePod;

    @Override
    public void runOpMode() {

        forwardPod = hardwareMap.get(DcMotor.class, "forwardPod");
        strafePod = hardwareMap.get(DcMotor.class, "strafePod");

        forwardPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafePod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        forwardPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafePod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            // read ticks directly
            int forwardTicks = forwardPod.getCurrentPosition();
            int strafeTicks  = strafePod.getCurrentPosition();

            telemetry.addData("Forward Pod Ticks", forwardTicks);
            telemetry.addData("Strafe Pod Ticks", strafeTicks);
            telemetry.update();
        }
    }
}
