package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OlsonHardware {
    public DcMotor RF;
    public DcMotor LF;
    public DcMotor RB;
    public DcMotor LB;

    public static double maxSpeed = 0.85;
    private static Hardware myInstance = null;
    public static Hardware getInstance(){
        if(myInstance == null) {
            myInstance = new Hardware();


        }
        return myInstance;
    }

    public void init(HardwareMap hwMap){


    }


}
