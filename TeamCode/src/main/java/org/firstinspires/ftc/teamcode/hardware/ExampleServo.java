package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class ExampleServo {

    String name;
    Servo servo;

    public ExampleServo( HardwareMap hardwareMap, String configName ) {

        name = configName;

        servo = hardwareMap.servo.get( configName );
    }

    public void setPosition( double position ) {

        position = Range.clip( position, Servo.MIN_POSITION, Servo.MAX_POSITION );

        servo.setPosition( position );
    }
}
