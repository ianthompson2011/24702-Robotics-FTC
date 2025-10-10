package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Hardware {

    //create motors
    public DcMotor rf;
    public DcMotor lf;
    public DcMotor lb;
    public DcMotor rb;

    //create servo
    public Servo demoServo;

    //create maxSpeed variable
    public static double maxSpeed = 0.9;

    //create instance of hardware class
    private static Hardware myInstance = null;
    public static Hardware getInstance() {

        if (myInstance == null) {
            myInstance = new Hardware();
        }
        return myInstance;
    }

    public void init(HardwareMap hwMap) {
        //intialize motors
        rf = hwMap.get(DcMotor.class, "rf");
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setPower(0);

        lf = hwMap.get(DcMotor.class, "lf");
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setPower(0);

        lb = hwMap.get(DcMotor.class, "lb");
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setPower(0);

        rb = hwMap.get(DcMotor.class, "rb");
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setPower(0);

        //intialize servo
        demoServo = hwMap.get(Servo.class, "demoServo");
    }

    //create setPower method that allows power to be set to all four motors
    public void setPower(double fr, double br, double bl, double fl) {

        rf.setPower(Range.clip(fr, -maxSpeed, maxSpeed));
        lf.setPower(Range.clip(fl, -maxSpeed, maxSpeed));
        rb.setPower(Range.clip(br, -maxSpeed, maxSpeed));
        lb.setPower(Range.clip(bl, -maxSpeed, maxSpeed));
    }
}