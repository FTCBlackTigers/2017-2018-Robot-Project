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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class BT_Glyphs {
    /* Public OpMode members. */

    public Servo armServo = null;
    public Servo    clamps    = null;
    public DcMotor armMotor = null ;

    //TODO: define constants
    public static final double MIN_ARM_POS = 23;
    public static final double MAX_ARM_POS =-500 ;
    public static final double ARM_MANUAL_POWER = 0.5;
    public static final double ARM_AUTO_POWER = 0.2;
    public static final int ARM_HIGH_POS = 520 ;
    public static final int ARM_LOW_POS = 200 ;
    public static final int ARM_DOWN_POS  = 0 ;
    public static final double  CLAMPS_OPEN_POS =  0.5 ;
    public static final double CLAMPS_CLOSE_POS =  0.5 ;
    public static final double SERVO_HIGH_POS = 0.2 ;
    public static final double SERVO_LOW_POS = 0.7 ;
    public static final double SERVO_DOWN_POS  = 0 ;
    public static final double SERVO_LIFT_POS = 0.7 ;
    public static final double SERVO_INTERVAL =0.01;
    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private OpMode callerOpmode ;

    /* Constructor */
    public BT_Glyphs(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, OpMode callerOpmode) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        this.callerOpmode =callerOpmode;
        // Define and Initialize Motors
        armServo = hwMap.get(Servo.class, "armServo");
//        clamps = hwMap.get(Servo.class, "clampsServo");
        armMotor = hwMap.get(DcMotor.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        moveServo(SERVO_DOWN_POS);
//        clamps.setPosition(CLAMPS_OPEN_POS);
    }

    public void moveArm(int pos) {
        armMotor.setTargetPosition(pos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ARM_AUTO_POWER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void armHigh(){
        moveArm(ARM_HIGH_POS);
        while (armMotor.getCurrentPosition() <= ARM_HIGH_POS*0.5){
        }
        moveServo(SERVO_HIGH_POS);

    }
    public void armLow(){
        moveArm(ARM_LOW_POS);
        moveServo(SERVO_LOW_POS);
    }
    public void armDown(){
        moveArm(ARM_DOWN_POS);
        while (armMotor.getCurrentPosition() >= ARM_DOWN_POS/0.5){
        }
        moveServo(SERVO_DOWN_POS);
    }

    public void moveServo (double pos){
        armServo.setPosition(pos);
    }

    public void teleopMotion(Gamepad gamepad, Telemetry telemetry){
        final double JOYSTICK_THRESHOLD=0.8;
        double armMotorPower;
        double armServoPower;

        boolean catchGlyphs = gamepad.left_bumper;
        boolean releaseGlyphs= gamepad.right_bumper;
        boolean glyphsHigh = gamepad.y;
        boolean glyphsDown = gamepad.a;
        boolean glyphsLow = gamepad.b;

        // Handle manual arm control
        armMotorPower = -gamepad.left_stick_y;
        if ((Math.abs(armMotorPower) < JOYSTICK_THRESHOLD) || (MAX_ARM_POS>armMotor.getCurrentPosition())||(MIN_ARM_POS<armMotor.getCurrentPosition())){
            armMotorPower=0;
        }
        else {
            armMotorPower=Math.signum(armMotorPower)*ARM_MANUAL_POWER;
        }
        telemetry.addData("arm pos: ", armMotor.getCurrentPosition());
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(armMotorPower);

        // Handle manual servo control
        armServoPower = -gamepad.right_stick_y;
        if (Math.abs(armServoPower) < JOYSTICK_THRESHOLD){
            armServoPower=0;
        }
        if(armServoPower > 0 && armServo.getPosition() < 1) {
            moveServo(armServo.getPosition()+SERVO_INTERVAL);
        }
        else if (armServoPower < 0 && armServo.getPosition() > 0) {
            moveServo(armServo.getPosition()-SERVO_INTERVAL);
        }
        telemetry.addData("servo pos: ", armServo.getPosition());

        // Handle automatic operations
        //clamps system
        if (catchGlyphs){
            catchGlyphs();
            telemetry.addData("op: ","catch glyphs");
        }
        else if (releaseGlyphs){
            releaseGlyphs();
            telemetry.addData("op: ","release glyphs");
        }

        //glyphs arm system
        if (glyphsHigh){
            armHigh();
            telemetry.addData("op: ","arm high");
        }
        else if (glyphsLow){
            armLow();
            telemetry.addData("op: ","arm low");
        }
        else if (glyphsDown){
            armDown();
            telemetry.addData("op: ","arm down");
        }
    }

    public void catchGlyphs (){
//        clamps.setPosition(CLAMPS_OPEN_POS);
    }
    public void releaseGlyphs (){
//        clamps.setPosition(CLAMPS_CLOSE_POS);
    }
 }

