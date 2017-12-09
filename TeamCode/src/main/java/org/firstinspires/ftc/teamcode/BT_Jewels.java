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

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

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

public class BT_Jewels {

    public enum JewelColor {
        UNKNOWN,BLUE,RED;
    }

    /* Public OpMode members. */
    public Servo    jewelArm     = null;
    public Servo    jewelFinger    = null;


    //TODO: define constants
    public static final double JEWEL_ARM_START  =  0.5 ;
    public static final double JEWEL_FINGER_START  =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    public static final double JEWEL_FINGER_MID  =  0.5 ;
    public static final double JEWEL_FINGER_RIGHT    =  0.45 ;
    public static final double JEWEL_FINGER_LEFT  = -0.45 ;
    public static final double ARM_UP_INTERVAL =  0.1 ;

    private ElapsedTime runtimeJ = new ElapsedTime();
    static final double WAIT_FOR_COLOR = 3000;
    static final long WAIT_INTERVAL = 500;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public BT_Jewels(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        jewelArm = hwMap.get(Servo.class, "jewelArm");
        jewelFinger = hwMap.get(Servo.class, "jewelFinger");

        jewelArm.setPosition(JEWEL_ARM_START);
        jewelFinger.setPosition(JEWEL_FINGER_START);
    }
    public void armDown(){
        jewelFinger.setPosition(JEWEL_FINGER_MID);
        jewelArm.setPosition(ARM_DOWN_POWER);
    }
    public void armUp(){
        jewelArm.setPosition(ARM_UP_POWER);
        jewelFinger.setPosition(JEWEL_FINGER_START);
    }
    public void fingerRight(){
        jewelFinger.setPosition(JEWEL_FINGER_RIGHT);
    }
    public void fingerLeft(){
        jewelFinger.setPosition(JEWEL_FINGER_LEFT);
    }
    public JewelColor getJewelColor(){
        //TODO
        return JewelColor.BLUE;
    }
    public void moveJewel(JewelColor targetColor , LinearOpMode callerOpmode){
        JewelColor jewelColor;
        armDown();
        jewelColor = getJewelColor();
        while ((jewelColor == JewelColor.UNKNOWN) && (runtimeJ.time()< WAIT_FOR_COLOR)) {
            callerOpmode.sleep(WAIT_INTERVAL);
            jewelColor = getJewelColor();
            jewelArm.setPosition(jewelArm.getPosition()+ARM_UP_INTERVAL);
        }
        if (jewelColor != JewelColor.UNKNOWN) {
            if (jewelColor == targetColor) {
                fingerLeft();
            } else {
                fingerRight();
            }
        }
        armUp();
    }
}

