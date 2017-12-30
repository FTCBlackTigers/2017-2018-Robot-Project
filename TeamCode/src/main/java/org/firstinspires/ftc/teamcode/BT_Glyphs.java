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
import com.qualcomm.robotcore.util.ElapsedTime;

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

    //TODO: define constantsS
    public static final double ARM_POWER = 0.5;
    public static final double  CLAMPS_OPEN_POS =  0.5 ;
    public static final double CLAMPS_CLOSE_POS =  0.5 ;
    public static final int ARM_HIGH_POS = 90 ;
    public static final int ARM_LOW_POS = 45 ;
    public static final int ARM_DOWN_POS  = 0 ;
    public static final double SERVO_HIGH_POS = 0.2 ;
    public static final double SERVO_LOW_POS = 0.7 ;
    public static final double SERVO_DOWN_POS  = 0.2 ;
    public static final double SERVO_LIFT_POS = 0.7 ;
    public static final double SERVO_LIFT_INTERVAL = 0.025 ;
    public static final double ARM_LIFT_INTERVAL = 9 ;
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
//        armServo = hwMap.get(Servo.class, "armServo");
//        clamps = hwMap.get(Servo.class, "clampsServo");
        armMotor = hwMap.get(DcMotor.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

//        armServo.setPosition(SERVO_DOWN_POS);
//        clamps.setPosition(CLAMPS_OPEN_POS);
    }
//    private void servoChangePos(double pos) {
//        armServo.setPosition(pos);
//    }

//    public void servoLow() {
//        servoChangePos(SERVO_LOW_POS);
//    }

//    public void servoHigh(){
//        servoChangePos(SERVO_HIGH_POS);
//    }

    public void armLow() {
        int motorTarget = armMotor.getCurrentPosition();
//        double servoTarget = armServo.getPosition();
        while (armMotor.getCurrentPosition() < ARM_LOW_POS ){
            motorTarget += ARM_LIFT_INTERVAL;
            armMotor.setTargetPosition(motorTarget);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(ARM_POWER);
            while (armMotor.getCurrentPosition() < motorTarget ) {
            }
            armMotor.setPower(0);
//            if (armServo.getPosition() < SERVO_LIFT_POS){
//            servoTarget += SERVO_LIFT_INTERVAL;
//            armServo.setPosition(servoTarget);
//            while (armServo.getPosition() < servoTarget ) {
//            }

        }
    }
    public void armUp (){
        armLow();
        armMotor.setTargetPosition(ARM_HIGH_POS);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ARM_POWER);
        while (armMotor.getCurrentPosition() < ARM_HIGH_POS  ) {
        }
        armMotor.setPower(0);

//        int newLeftTarget;
//        int newRightTarget;
//        ElapsedTime runtime =new ElapsedTime();
//        // Determine new target position, and pass to motor controller
//        newLeftTarget = frontLeftDrive.getCurrentPosition() + (int)(leftCm * COUNTS_PER_CM);
//        newRightTarget = frontRightDrive.getCurrentPosition() + (int)(rightCm * COUNTS_PER_CM);
//        frontLeftDrive.setTargetPosition(newLeftTarget);
//        frontRightDrive.setTargetPosition(newRightTarget);
//
//        // Turn On RUN_TO_POSITION
//        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        // reset the timeout time and start motion.
//        runtime.reset();
//        frontLeftDrive.setPower(Math.abs(speed));
//        frontRightDrive.setPower(Math.abs(speed));
//
//        // keep looping while we are still active, and there is time left, and both motors are running.
//        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//        // its target position, the motion will stop.  This is "safer" in the event that the robot will
//        // always end the motion as soon as possible.
//        // However, if you require that BOTH motors have finished their moves before the robot continues
//        // onto the next step, use (isBusy() || isBusy()) in the loop test.
//        while ((runtime.time() < timeoutMs) &&
//                (frontLeftDrive.isBusy() && frontRightDrive.isBusy())) {
//
//        }
//
//        // Stop all motion;
//        frontLeftDrive.setPower(0);
//        frontRightDrive.setPower(0);
//
//        // Turn off RUN_TO_POSITION
//        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void armTeleopMotion (Gamepad gamepad){
        final double JOYSTICK_THRESHOLD=0.1;
        double armMotorPower;
        double armServoPower;
        armMotorPower = -gamepad.left_stick_y;
        if (Math.abs(armMotorPower) < JOYSTICK_THRESHOLD){
            armMotorPower=0;
        }
        armServoPower = -gamepad.right_stick_y;
        if (Math.abs(armServoPower) < JOYSTICK_THRESHOLD){
            armServoPower=0;
        }
        armMotor.setPower(armMotorPower);
        if(armServoPower > 0 && armServo.getPosition() < 1) {
            armServo.setPosition(armServo.getPosition()+0.01);
        }
        else if (armServoPower < 0 && armServo.getPosition() > 0) {
            armServo.setPosition(armServo.getPosition()-0.01);
        }

    }
//    }
    public void armDown (){
        armServo.setPosition(ARM_DOWN_POS);
    }
    public void catchGlyphs (){
        clamps.setPosition(CLAMPS_OPEN_POS);
    }
    public void releaseGlyphs (){
        clamps.setPosition(CLAMPS_CLOSE_POS);
    }
 }

