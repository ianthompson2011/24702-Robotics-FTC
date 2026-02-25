package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Hardware {

    public DcMotor rf;
    public DcMotor lf;
    public DcMotor rb;
    public DcMotor lb;
    public DcMotorEx ls;
    public DcMotorEx rs;
    public Servo demoServo1;

    public DcMotor it;

    public static Pose lastAutoPose = null;

    public static double maxSpeed = 0.7;
    // slightly lowered speed to prevent feeling of being overly reactive
    private static Hardware myInstance = null;
    public static Hardware getInstance(){
        if(myInstance == null) {
            myInstance = new Hardware();
        }
        return myInstance;
    }
    public void init(HardwareMap hwMap){

        // motors for wheels
        // in this order, left front, right front, right back, left back

        lf = hwMap.get(DcMotor.class, "cm2");
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setPower(0);

        rf = hwMap.get(DcMotor.class, "cm3");
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setPower(0);

        rb = hwMap.get(DcMotor.class, "cm0");
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setPower(0);

        lb = hwMap.get(DcMotor.class, "cm1");
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setPower(0);

        // motors for shooters

        ls = hwMap.get(DcMotorEx.class, "em0");
        ls.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        ls.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ls.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ls.setPower(0);

        rs = hwMap.get(DcMotorEx.class, "em1");
        rs.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rs.setDirection(DcMotorSimple.Direction.REVERSE);
        rs.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rs.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rs.setPower(0);

        // motor for intake, controls rubber bands

        it = hwMap.get(DcMotor.class, "em2");
        it.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        it.setDirection(DcMotorSimple.Direction.REVERSE);
        it.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        it.setPower(0);

        // servos to push balls up to rs and ls motors
        demoServo1 = hwMap.get(Servo.class, "cs0");
    }
    public void setPower(double fl, double fr, double bl, double br){
        lf.setPower(Range.clip(fl, -maxSpeed, maxSpeed));
        rf.setPower(Range.clip(fr, -maxSpeed, maxSpeed));
        lb.setPower(Range.clip(bl, -maxSpeed, maxSpeed));
        rb.setPower(Range.clip(br, -maxSpeed, maxSpeed));
    }
}