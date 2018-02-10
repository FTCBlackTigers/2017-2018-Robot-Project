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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

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
public class BT_MecanumDrive {
    public static double lastVD = 0        ;
    public static final double DELTA_ACCELERATION = 0.13;
    public static final double TELEOP_DRIVE_SPEED = 0.8;
    public static final double SLOW_SPEED = 0.3;
    public static final double TURN_SPEED = 0.2;
    public enum DriveDirection {
        FORWARD,BACKWARD,RIGHT,LEFT;
    }
    public double frontLeftFactor = 1;
    public double frontRightFactor = 1;
    public double rearLeftFactor = 1;
    public double rearRightFactor = 1;

    //invert joystick values to motion
    public static class Motion {
        // Robot speed [-1, 1].
        public final double vD;
        // Robot angle while moving [0, 2pi].
        public final double thetaD;
        // Speed for changing direction [-1, 1].
        public final double vTheta;


        /**
         * Sets the motion to the given values.
         */
        public Motion(double vD, double thetaD, double vTheta) {
            this.vD = vD;
            this.thetaD = thetaD;
            this.vTheta = vTheta;
        }
    }

    /**
     * Gets the motion vector from the joystick values.
     * @param leftStickX The left joystick X.
     * @param leftStickY The left joystick Y.
     * @param rightStickX The right joystick X.
     * @param rightStickY The right joystick Y.
     * @return The Mecanum motion vector.
     */
    public static Motion joystickToMotion(double leftStickX,
                                                     double leftStickY,
                                                     double rightStickX,
                                                     double rightStickY, double rightTrigger,
                                          double curretAngle) {
        final double JOYSTICK_THRESHOLD = 0.2;
        boolean glyphIn = rightTrigger > 0.5;
        double leftX = leftStickX;
        double leftY = -leftStickY;
        double rightX = rightStickX;
        if (Math.abs(leftX) < JOYSTICK_THRESHOLD){
            leftX=0;
        }
        if (Math.abs(leftY) < JOYSTICK_THRESHOLD){
            leftY=0;
        }
        if (Math.abs(rightX) < JOYSTICK_THRESHOLD){
            rightX=0;
        }
        BT_Status.addLine("leftX: " + leftX);
        BT_Status.addLine("leftY: " + leftY);
        BT_Status.addLine("rightX: " + rightX);

        double vD = Math.min(Math.sqrt(Math.pow(leftX, 2) +
                        Math.pow(leftY, 2)), 1);
        if ((vD > lastVD + DELTA_ACCELERATION) && (lastVD + DELTA_ACCELERATION < 0.6)) {
            vD = lastVD + DELTA_ACCELERATION ;
        }
        lastVD = vD;
        vD = vD*(glyphIn ? SLOW_SPEED : TELEOP_DRIVE_SPEED);
        double thetaD = Math.atan2(leftX,leftY);
        BT_Status.addLine("thetaD: "+thetaD);
        double radAngle = curretAngle*Math.PI/180;
        //driving by driver's view
        thetaD += radAngle;
        while (thetaD > Math.PI)  thetaD -=  Math.PI * 2;
        while (thetaD <= - Math.PI) thetaD +=  Math.PI * 2;
        BT_Status.addLine("thetaD 2: "+thetaD);
        double vTheta = Math.abs(rightX) < 0.8 ? rightX*TURN_SPEED : rightX;

        return new Motion(vD, thetaD, vTheta);
    }
    public static class Wheels {
        // The mecanum wheels powers.
        public double frontLeft;
        public double frontRight;
        public double backLeft;
        public double backRight;

        /**
         * Sets the wheels to the given values.
         */
        public Wheels(double frontLeft, double frontRight,
                      double backLeft, double backRight) {
            List<Double> powers = Arrays.asList(frontLeft, frontRight,
                    backLeft, backRight);
            clampPowers(powers);

            this.frontLeft = powers.get(0);
            this.frontRight = powers.get(1);
            this.backLeft = powers.get(2);
            this.backRight = powers.get(3);
        }

        /**
         * Scales the wheel powers by the given factor.
         * @param scalar The wheel power scaling factor.
         */
        public void scaleWheelPower(double scalar) {
            frontLeft*=scalar;
            frontRight*=scalar;
            backLeft*=scalar;
            backRight*=scalar;

        }
    }

    /**
     * Gets the wheel powers corresponding to desired motion.
     * @param motion The Mecanum motion vector.
     * @return The wheels with clamped powers. [-1, 1]
     */
    public static Wheels motionToWheels(Motion motion) {
        double vD = motion.vD;
        double thetaD = motion.thetaD;
        double vTheta = motion.vTheta;

        double frontLeft = vD * Math.sin(thetaD + Math.PI / 4) + vTheta;
        double frontRight  = vD * Math.cos(thetaD + Math.PI / 4) - vTheta;
        double backLeft = vD * Math.cos(thetaD + Math.PI / 4) + vTheta;
        double backRight = vD * Math.sin(thetaD + Math.PI / 4) - vTheta;
        Wheels wheels = new Wheels(frontLeft, frontRight, backLeft, backRight);
        wheels.scaleWheelPower(vD>0?Math.abs(vD):Math.abs(vTheta));
        return wheels;
    }

    /**
     * Clamps the motor powers while maintaining power ratios.
     * @param powers The motor powers to clamp.
     */
    private static void clampPowers(List<Double> powers) {
        double minPower = Collections.min(powers);
        double maxPower = Collections.max(powers);
        double maxMag = Math.max(Math.abs(minPower), Math.abs(maxPower));

        for (int i = 0; i < powers.size(); i++) {
            powers.set(i, powers.get(i) / maxMag);
        }
    }

    /* Public OpMode members. */
    public DcMotor  frontLeftDrive   = null;
    public DcMotor  frontRightDrive  = null;
    public DcMotor  rearLeftDrive   = null;
    public DcMotor  rearRightDrive  = null;
    public BT_Gyro  gyro = new BT_Gyro();

    static final double     COUNTS_PER_MOTOR_REV    = 28 ;
    static final double     DRIVE_GEAR_REDUCTION    = 19.2 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CM       = 10.16 ;     // For figuring circumference
    static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);
    static final double     AUTO_DRIVE_SPEED = 0.4;
    static final double      AUTO_TURN_SPEED  = 0.3;


    static final double     THRESHOLD = 1;
    static final double     P_TURN_COEFF            = 0.1;

    ElapsedTime runtime = new ElapsedTime();
    private OpMode callerOpmode;

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public BT_MecanumDrive(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, OpMode callerOpmode) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        this.callerOpmode = callerOpmode;

        // Define and Initialize Motors
        frontLeftDrive  = hwMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hwMap.get(DcMotor.class, "frontRightDrive");
        rearLeftDrive = hwMap.get(DcMotor.class, "rearLeftDrive");
        rearRightDrive = hwMap.get(DcMotor.class, "rearRightDrive");

        //set motors dir
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Initiate the gyro
        gyro.init(hwMap);
        readMotorFactor(this.callerOpmode.telemetry);
    }
    public void readMotorFactor(Telemetry telemetry){
        try {
            InputStreamReader inputStreamReader = new InputStreamReader(hwMap.appContext.openFileInput("DriveConfig.txt"));
            BufferedReader bufferedReader = new BufferedReader(inputStreamReader);
            StringBuilder stringBuilder;
            stringBuilder = new StringBuilder();
            stringBuilder.append(bufferedReader.readLine());
//            StringBuilder stringBuilder;
//            stringBuilder = new StringBuilder();
//            stringBuilder.append(bufferedReader.readLine().split("\n")[0]);
//            frontLeftFactor = Double.parseDouble(stringBuilder.toString());
//            stringBuilder = new StringBuilder();
//            stringBuilder.append(bufferedReader.readLine().split("\n")[1]);
//            rearLeftFactor = Double.parseDouble(stringBuilder.toString());
//            stringBuilder = new StringBuilder();
//            stringBuilder.append(bufferedReader.readLine().split("\n")[2]);
//            frontRightFactor = Double.parseDouble(stringBuilder.toString());
//            stringBuilder = new StringBuilder();
//            stringBuilder.append(bufferedReader.readLine().split("\n")[3]);
//            rearRightFactor = Double.parseDouble(stringBuilder.toString());
            telemetry.setAutoClear(false);
            telemetry.addLine(stringBuilder.toString());
            telemetry.update();
        } catch (java.io.IOException e) {
            telemetry.addLine("file not found");
            telemetry.update();
        }
       // telemetry.addData("drives","%f, %f, %f, %f",frontLeftFactor,rearLeftFactor,frontRightFactor,rearRightFactor);
        //telemetry.update();

    }


    public void runWithoutEncoders (){
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void move (double distCm , DriveDirection direction,  double timeoutS, Telemetry telemetry ){
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        encoderDrive( AUTO_DRIVE_SPEED, distCm, direction , timeoutS, telemetry );

    }

    public void turn (double degrees, double timeoutMs, Telemetry telemetry,boolean isAuto) {
        double rightSpeed, leftSpeed;
        double steer;
        double error = getError(degrees);
        double t;
        runtime.reset();
        boolean isActive = true;

        // keep looping while we are still active, and not on heading.
        while((Math.abs(error) > THRESHOLD) && (runtime.milliseconds() < timeoutMs) &&
                isActive) {
            while (Math.abs(error) > THRESHOLD && (isActive)) {
                if (isAuto){
                    isActive = ((LinearOpMode)callerOpmode).opModeIsActive();
                }
                // Update telemetry & Allow time for other processes to run.
                steer = getSteer(error, P_TURN_COEFF);
                rightSpeed = AUTO_TURN_SPEED * steer;
                if(rightSpeed>0&& rightSpeed<0.1)
                    rightSpeed=0.1;
                else if(rightSpeed<0&&rightSpeed>-0.1)
                    rightSpeed=-0.1;
                leftSpeed = -rightSpeed;

                frontLeftDrive.setPower(leftSpeed);
                frontRightDrive.setPower(rightSpeed);
                rearLeftDrive.setPower(leftSpeed);
                rearRightDrive.setPower( rightSpeed);

                error = getError(degrees);
                telemetry.addData("Error", error);
                telemetry.addLine("angle : " + gyro.getAngle());
                telemetry.update();
            }
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            rearLeftDrive.setPower(0);
            rearRightDrive.setPower(0);

            t= runtime.milliseconds();
            while (runtime.milliseconds() < t + 300 && isActive){
                if (isAuto){
                    isActive = ((LinearOpMode)callerOpmode).opModeIsActive();
                }
                error = getError(degrees);
                telemetry.addData("Error", error);
                telemetry.addLine("angle : " + gyro.getAngle());
                telemetry.update();
            }
        }
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);
    }
    public void teleopTurn (double degrees, Telemetry telemetry){
        double rightSpeed, leftSpeed;
        double steer;
        double error = getError(degrees);
        double t;
        runtime.reset();
        boolean isActive = true;

        // keep looping while we are still active, and not on heading.
        if ((Math.abs(error) > THRESHOLD) && isActive) {
                // Update telemetry & Allow time for other processes to run.
                steer = getSteer(error, P_TURN_COEFF);
                rightSpeed = AUTO_TURN_SPEED * steer;
                if(rightSpeed>0&& rightSpeed<0.1)
                    rightSpeed=0.1;
                else if(rightSpeed<0&&rightSpeed>-0.1)
                    rightSpeed=-0.1;
                leftSpeed = -rightSpeed;

                frontLeftDrive.setPower(leftSpeed);
                frontRightDrive.setPower(rightSpeed);
                rearLeftDrive.setPower(leftSpeed);
                rearRightDrive.setPower( rightSpeed);

                error = getError(degrees);
                telemetry.addData("Error", error);
                telemetry.addLine("angle : " + gyro.getAngle());
                telemetry.update();
        }
        else {
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            rearLeftDrive.setPower(0);
            rearRightDrive.setPower(0);
        }
    }
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getAngle();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void teleopDrive(Gamepad gamepad, Telemetry telemetry) {
        boolean turnCloseCrypto = gamepad.a;
        boolean turnSideCrypto = gamepad.b || gamepad.x;
        boolean forwDrive = gamepad.dpad_up;
        boolean rightDrive = gamepad.dpad_right;
        boolean leftDrive = gamepad.dpad_left;
        boolean backDrive = gamepad.dpad_down;
        boolean resetGyro = gamepad.start;
        double robotAngle = 0;
        if (gamepad.right_bumper ){
            robotAngle = gyro.getAngle();
        }
        Motion motion = joystickToMotion(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.right_stick_x, gamepad.right_stick_y,
                gamepad.right_trigger, robotAngle);
        if (rightDrive){
            motion = new Motion(0.3, Math.PI/2, 0);
        }
        else if (leftDrive){
            motion = new Motion(0.3, -Math.PI/2, 0);
        }
        else if(backDrive){
            motion = new Motion(0.3, Math.PI, 0);
        }
        else if(forwDrive) {
            motion = new Motion(0.3, 0, 0);
        }
        Wheels wheels = motionToWheels(motion);
        if (turnCloseCrypto){
            teleopTurn(BT_FieldSetup.closeCryptobox, telemetry);
        }
        else if (turnSideCrypto){
            teleopTurn(BT_FieldSetup.sideCryptobox, telemetry);
        }
        else {
            frontLeftDrive.setPower(wheels.frontLeft/*frontLeftFactor*/);
            frontRightDrive.setPower(wheels.frontRight/*frontRightFactor*/);
            rearLeftDrive.setPower(wheels.backLeft/*rearLeftFactor*/);
            rearRightDrive.setPower(wheels.backRight/*rearRightFactor*/);
            telemetry.addLine("DRIVE");
            telemetry.addLine(" front left: " + wheels.frontLeft + ", " + frontLeftDrive.getCurrentPosition());
            telemetry.addLine(" front right : " + wheels.frontRight + ", " + frontRightDrive.getCurrentPosition());
            telemetry.addLine(" rear left : " + wheels.backLeft + ", " + rearLeftDrive.getCurrentPosition());
            telemetry.addLine(" rear right : " + wheels.backRight + ", " + rearRightDrive.getCurrentPosition());
            telemetry.addLine(" angle : " + gyro.getAngle());
        }
        if (resetGyro){
            gyro.init(hwMap);
        }

    }

    public void encoderDrive(double speed,
                             double distCm, DriveDirection direction,
                             double timeoutMs, Telemetry telemetry) {
        double robotAngle = gyro.getAngle();
        int newFrontLeftTarget = frontLeftDrive.getCurrentPosition();
        int newFrontRightTarget = frontRightDrive.getCurrentPosition();
        int newRearLeftTarget = rearLeftDrive.getCurrentPosition();
        int newRearRightTarget = rearRightDrive.getCurrentPosition();

        ElapsedTime runtime =new ElapsedTime();
            // Determine new target position, and pass to motor controller
        int ticks = (int)(distCm / Math.cos(Math.PI/4) * COUNTS_PER_CM);
        switch (direction){
            case FORWARD:
                newFrontLeftTarget += ticks ;
                newFrontRightTarget += ticks;
                newRearLeftTarget += ticks;
                newRearRightTarget += ticks;
                break;
            case BACKWARD:
                newFrontLeftTarget -= ticks ;
                newFrontRightTarget -= ticks;
                newRearLeftTarget -= ticks;
                newRearRightTarget -= ticks;
                break;
            case RIGHT:
                newFrontLeftTarget -= ticks ;
                newFrontRightTarget += ticks;
                newRearLeftTarget += ticks;
                newRearRightTarget -= ticks;
                break;
            case LEFT:
                newFrontLeftTarget += ticks ;
                newFrontRightTarget -= ticks;
                newRearLeftTarget -= ticks;
                newRearRightTarget += ticks;
                break;

        }
//        telemetry.addData("front left: " , ticks + "," + newFrontLeftTarget);
//        telemetry.addData("front right: " , ticks + "," + newFrontRightTarget);
//        telemetry.addData("rear left: " , ticks + "," + newRearLeftTarget);
//        telemetry.addData("rear right: " , ticks + "," + newRearRightTarget);

        frontLeftDrive.setTargetPosition(newFrontLeftTarget);
        frontRightDrive.setTargetPosition(newFrontRightTarget);
        rearLeftDrive.setTargetPosition(newRearLeftTarget);
        rearRightDrive.setTargetPosition(newRearRightTarget);

        // Turn On RUN_TO_POSITION
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        frontLeftDrive.setPower(Math.abs(speed));
        frontRightDrive.setPower(Math.abs(speed));
        rearLeftDrive.setPower(Math.abs(speed));
        rearRightDrive.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((runtime.milliseconds() < timeoutMs) && (((LinearOpMode)callerOpmode).opModeIsActive()) &&
                (frontLeftDrive.isBusy() && frontRightDrive.isBusy() && rearLeftDrive.isBusy() && rearRightDrive.isBusy())) {
//            telemetry.addData("front left: " ,newFrontLeftTarget + " , " + frontLeftDrive.getCurrentPosition());
//            telemetry.addData("front right: " ,newFrontRightTarget + " , " + frontRightDrive.getCurrentPosition());
//            telemetry.addData("rear left: " ,newRearLeftTarget + " , " + rearLeftDrive.getCurrentPosition());
//            telemetry.addData("rear right: " ,newRearRightTarget + " , " + rearRightDrive.getCurrentPosition());
            double steer = getSteer(getError(robotAngle), P_TURN_COEFF);
            if (direction == DriveDirection.BACKWARD){
                steer *= -1.0;
            }
            double leftSpeed = speed - steer;
            double rightSpeed = speed + steer;
            double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0) {
                leftSpeed /= max;
                rightSpeed /= max;
            }
//            frontLeftDrive.setPower(leftSpeed);
//            frontRightDrive.setPower(rightSpeed);
//            rearRightDrive.setPower(rightSpeed);
//            rearLeftDrive.setPower(leftSpeed);
        }

        // Stop all motion;
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void buttonDrive (Gamepad gamepad,Telemetry telemetry) {
        double speed = 1;
        if (gamepad.right_bumper){
            speed = -1;
        }
        if (gamepad.y){
            frontRightDrive.setPower(speed);
        }
        else {
            frontRightDrive.setPower(0);
        }
        if (gamepad.x){
            frontLeftDrive.setPower(speed);
        }
        else {
            frontLeftDrive.setPower(0);
        }
        if (gamepad.b){
            rearRightDrive.setPower(speed);
        }
        else {
            rearRightDrive.setPower(0);
        }
        if (gamepad.a){
            rearLeftDrive.setPower(speed);
        }
        else {
            rearLeftDrive.setPower(0);
        }
        telemetry.addData("front left: " , frontLeftDrive.getCurrentPosition());
        telemetry.addData("front right: " , frontRightDrive.getCurrentPosition());
        telemetry.addData("rear left: " , rearLeftDrive.getCurrentPosition());
        telemetry.addData("rear right: " , rearRightDrive.getCurrentPosition());
        telemetry.update();

    }

}
