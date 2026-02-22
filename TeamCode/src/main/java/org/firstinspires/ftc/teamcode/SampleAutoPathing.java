package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

public class SampleAutoPathing extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    public enum PathState {
        // start position to end position
        // drive > movement state
        // shoot > attempt to score artifact
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD
    }
    PathState pathstate;
    @Override
    public void init(){

    }
    @Override
    public void loop(){

    }
}
