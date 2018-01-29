
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static android.os.SystemClock.sleep;

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
    public NormalizedColorSensor colorSensor = null;


    public static final double JEWEL_ARM_START  =  0 ;
    public static final double JEWEL_FINGER_START  = 1.0 ;
    public static final double ARM_DOWN_POS = 0.65 ;
    public static final double JEWEL_FINGER_MID  = 0.35 ;
    public static final double JEWEL_FINGER_RIGHT    =  1.0 ;
    public static final double JEWEL_FINGER_LEFT  = 0 ;
    public static final double ARM_UP_INTERVAL =  0.1 ;

    private ElapsedTime runtimeJ = new ElapsedTime();
    private OpMode callerOpmode;

    static final double WAIT_FOR_COLOR = 3000;
    static final long WAIT_INTERVAL = 500;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public BT_Jewels(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, OpMode callerOpmode) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        this.callerOpmode = callerOpmode;

        // Define and Initialize Motors
        jewelArm = hwMap.get(Servo.class, "jewelArm");
        jewelFinger = hwMap.get(Servo.class, "jewelFinger");
        colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");


        jewelArm.setPosition(JEWEL_ARM_START);
        jewelFinger.setPosition(JEWEL_FINGER_START);
    }

    public void armDown(){
        jewelFinger.setPosition(JEWEL_FINGER_MID);
        for (double pos = JEWEL_ARM_START; pos<=ARM_DOWN_POS; pos+=0.05 ){
            jewelArm.setPosition(pos);
            ((LinearOpMode)callerOpmode).sleep(100);
        }
        jewelArm.setPosition(ARM_DOWN_POS);
    }

    public void armUp(){
//        while (jewelArm.getPosition() >= JEWEL_ARM_START){
//            jewelArm.setPosition(JEWEL_ARM_START);
//        }

        jewelArm.setPosition(JEWEL_ARM_START);
        sleep(500);
        jewelFinger.setPosition(JEWEL_FINGER_START);
    }

    public void fingerRight(){
        jewelFinger.setPosition(JEWEL_FINGER_RIGHT);
    }

    public void fingerLeft(){
        jewelFinger.setPosition(JEWEL_FINGER_LEFT);
    }

    public JewelColor getJewelColor(){
        float redValue;
        float blueValue;
        final float COLOR_DELTA = 0;
        JewelColor jewelColor = JewelColor.UNKNOWN;
        // Read the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        redValue = colors.red;
        blueValue = colors.blue;
        if (redValue-blueValue>COLOR_DELTA){
            jewelColor = JewelColor.RED;
        }
        else if (blueValue-redValue>COLOR_DELTA){
            jewelColor = JewelColor.BLUE;
        }

        return jewelColor;
    }

    public void moveJewel(JewelColor targetColor){
        if((((LinearOpMode)callerOpmode).opModeIsActive())) {
            JewelColor jewelColor;
            armDown();
            ((LinearOpMode) callerOpmode).sleep(1000);
            jewelColor = getJewelColor();
            while ((jewelColor == JewelColor.UNKNOWN) && (runtimeJ.milliseconds() < WAIT_FOR_COLOR)) {
                ((LinearOpMode) callerOpmode).sleep(WAIT_INTERVAL);
                jewelColor = getJewelColor();
                jewelArm.setPosition(jewelArm.getPosition() + ARM_UP_INTERVAL);
            }
            if (jewelColor != JewelColor.UNKNOWN) {
                BT_Status.addLine("color: " + jewelColor);
                if (jewelColor.equals(targetColor)) {
                    fingerRight();
                    BT_Status.addLine("dir : right ");
                } else {
                    fingerLeft();
                    BT_Status.addLine("dir : left ");
                }
            }
            ((LinearOpMode) callerOpmode).sleep(1000);
            armUp();
        }
    }
}

