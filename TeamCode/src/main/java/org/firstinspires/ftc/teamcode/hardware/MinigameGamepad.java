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

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.Gamepad;

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
public class MinigameGamepad
{
    Gamepad gamepad;

    boolean xButtonPressed = false;
    boolean yButtonPressed = false;
    boolean aButtonPressed = false;
    boolean bButtonPressed = false;
    boolean leftBumperPressed = false;
    boolean rightBumperPressed = false;

    /* Constructor */
    public MinigameGamepad( Gamepad gamepad ) {

        this.gamepad = gamepad;
    }

    public double getLeftStickY() {

        return gamepad.left_stick_y;
    }
    
    public double getLeftStickX() {

        return gamepad.left_stick_x;
    }
    
    public double getRightStickY() {

        return gamepad.right_stick_y;
    }
    
    public double getRightStickX() {

        return gamepad.right_stick_x;
    }

    public double getLeftTrigger() {

        return gamepad.left_trigger;
    }

    public double getRightTrigger() {

        return gamepad.right_trigger;
    }

    public boolean isXPressed() {

        if( gamepad.x && !xButtonPressed ) {
            xButtonPressed = true;
            return true;

        } else {
            if( !gamepad.x ) {
                xButtonPressed = false;
            }

            return false;
        }
    }

    public boolean isYPressed() {

        if( gamepad.y && !yButtonPressed ) {
            yButtonPressed = true;
            return true;
        } else {
            if( !gamepad.y ) {
                yButtonPressed = false;
            }
            return false;
        }
    }
    
    public boolean isAPressed() {

        if( gamepad.a && !aButtonPressed ) {
            aButtonPressed = true;
            return true;
        } else {
            if( !gamepad.a ) {
                aButtonPressed = false;
            }
            return false;
        }
    }

    public boolean isBPressed() {

        if( gamepad.b && !bButtonPressed ) {
            bButtonPressed = true;
            return true;
        } else {
            if( !gamepad.b ) {
                bButtonPressed = false;
            }
            return false;
        }
    }

    public boolean isLeftBumperPressed() {

        if (gamepad.left_bumper && !leftBumperPressed) {
            leftBumperPressed = true;
            return true;
        } else {
            if (!gamepad.left_bumper) {
                leftBumperPressed = false;
            }
            return false;
        }
    }
    
    public boolean isRightBumperPressed() {

        if (gamepad.right_bumper && !rightBumperPressed) {
            rightBumperPressed = true;
            return true;
        } else {
            if (!gamepad.right_bumper) {
                rightBumperPressed = false;
            }
            return false;
        }
    }
        
    public boolean isDpadLeftPressed() {

        return gamepad.dpad_left;
    }
 
    public boolean isDpadRightPressed() {

        return gamepad.dpad_right;
    }

    public boolean isDpadUpPressed() {

        return gamepad.dpad_up;
    }

    public boolean isDpadDownPressed() {

        return gamepad.dpad_down;
    }
 }

