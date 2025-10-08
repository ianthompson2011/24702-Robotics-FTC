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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *

 */
public class SampleColorSensor
{
    ColorSensor colorSensor;
    Telemetry telemetry;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float[] hsvValues = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    final float[] values = hsvValues;

    static final double HSV_RED_LOW_RANGE = 17.0;
    static final double HSV_RED_HIGH_RANGE = 32.0;
    static final double HSV_BLUE_LOW_RANGE = 208.0;
    static final double HSV_BLUE_HIGH_RANGE = 225.0;

     // bPrevState and bCurrState represent the previous and current state of the button.
    boolean bPrevState = false;

    // bLedOn represents the state of the LED.
    boolean bLedOn = true;

    View relativeLayout;

    /* Constructor */
    public SampleColorSensor( HardwareMap hardwareMap, String deviceName, Telemetry telemetry ) {

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.colorSensor.get( deviceName );

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        this.telemetry = telemetry;
    }

    public void updateColor( boolean bCurrState ) {
        // check for button state transitions.
        if (bCurrState && (bCurrState != bPrevState))  {

            // button is transitioning to a pressed state. So Toggle LED
            bLedOn = !bLedOn;
            colorSensor.enableLed(bLedOn);
        }

        // update previous state variable.
        bPrevState = bCurrState;

        // convert the RGB values to HSV values.
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("LED", bLedOn ? "On" : "Off");
        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Argb", colorSensor.argb());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("Saturation", hsvValues[1]);
        telemetry.addData("Value", hsvValues[2]);
        telemetry.addData( "IsRed", HSV_RED_LOW_RANGE <= hsvValues[0] && hsvValues[0] <= HSV_RED_HIGH_RANGE ? "true" : "false" );
        telemetry.addData( "IsBlue", HSV_BLUE_LOW_RANGE <= hsvValues[0] && hsvValues[0] <= HSV_BLUE_HIGH_RANGE ? "true" : "false" );


        telemetry.update();

        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        relativeLayout.post(() -> relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values)));

    }
 }

