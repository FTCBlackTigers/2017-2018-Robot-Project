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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="TurnTest", group="Auto")

public class BT_AutoTurnTest extends LinearOpMode {
    // Declare OpMode members.
    protected ElapsedTime runtime = new ElapsedTime();
    protected BT_Hardware robot = new BT_Hardware();
    static final double WAIT_FOR_VUMARK = 3000;
    protected static double LEFT_DRIVE_DIST ;
    protected static double CENTER_DRIVE_DIST ;
    protected static double RIGHT_DRIVE_DIST ;
    protected static double CRYPTO_DIST  ;
    protected static double CLOSE_CRYPTO_ANGLE ;
    protected static double SIDE_CRYPTO_ANGLE  ;
    protected static double FINAL_ROBOT_ANGLE ;
    protected static BT_Jewels.JewelColor TARGET_JEWEL_COLOR ;

    public void initAutoConstants(){
    }
    @Override
    public void runOpMode() {
   robot.init(hardwareMap,this);
        telemetry.setAutoClear(true);
        BT_Status.cleanStatus();
        BT_Status.addLine("Robot Initialized");
        telemetry.addData("Status", BT_Status.getStatusLine());
        telemetry.update();
        waitForStart();
        robot.drive.move(100, BT_MecanumDrive.DriveDirection.FORWARD, 10000, telemetry);
        robot.drive.turn(90,5000,telemetry,true);
        telemetry.addLine(Double.toString(robot.drive.gyro.getAngle()));
        robot.drive.move(100, BT_MecanumDrive.DriveDirection.FORWARD, 10000, telemetry);
        robot.drive.turn(180,5000,telemetry,true);
        telemetry.addLine(Double.toString(robot.drive.gyro.getAngle()));
        robot.drive.move(100, BT_MecanumDrive.DriveDirection.FORWARD, 10000, telemetry);
        robot.drive.turn(-90,5000,telemetry,true);
        telemetry.addLine(Double.toString(robot.drive.gyro.getAngle()));
        robot.drive.move(100, BT_MecanumDrive.DriveDirection.FORWARD, 10000, telemetry);
        robot.drive.turn(0,5000,telemetry,true);
        telemetry.addLine(Double.toString(robot.drive.gyro.getAngle()));
        telemetry.update();
        sleep(1000000);
    }



}
