package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Hardware {

    public DcMotor rf;
    public DcMotor lf;
    public DcMotor rb;
    public DcMotor lb;
    public DcMotor ls;
    public DcMotor rs;
    public Servo demoServo;
    public static double maxSpeed = 0.9;
    private static Hardware myInstance = null;
    public static Hardware getInstance(){
     if(myInstance == null) {
         myInstance = new Hardware();


    }
        return myInstance;
}
public void init(HardwareMap hwMap){
  //this wiil initalize motors

    lf = hwMap.get(DcMotor.class, "cm0");
    lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    lf.setDirection(DcMotorSimple.Direction.REVERSE);
    lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    lf.setPower(0);

    rf = hwMap.get(DcMotor.class, "cm1");
    rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rf.setPower(0);

    rb = hwMap.get(DcMotor.class, "cm2");
    rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rb.setPower(0);

    lb = hwMap.get(DcMotor.class, "cm3");
    lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    lb.setDirection(DcMotorSimple.Direction.REVERSE);
    lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    lb.setPower(0);

    ls = hwMap.get(DcMotor.class, "em0");
    ls.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    ls.setDirection(DcMotorSimple.Direction.REVERSE);
    ls.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    ls.setPower(0);

    rs = hwMap.get(DcMotor.class, "em1");
    rs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rs.setDirection(DcMotorSimple.Direction.REVERSE);
    rs.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rs.setPower(0);

    //initialize Servo
    demoServo = hwMap.get(Servo.class, "cs0");
}
public void setPower(double fr, double br, double bl, double fl){
    rf.setPower(Range.clip(fr, -maxSpeed, maxSpeed));
    lf.setPower(Range.clip(fl, -maxSpeed, maxSpeed));
    rb.setPower(Range.clip(br, -maxSpeed, maxSpeed));
    lb.setPower(Range.clip(bl, -maxSpeed, maxSpeed));

    }
}
//time based: uses time, does not account for slippage
// encoder: counts ticks to get to position, slipage
//odometry corrrects position
