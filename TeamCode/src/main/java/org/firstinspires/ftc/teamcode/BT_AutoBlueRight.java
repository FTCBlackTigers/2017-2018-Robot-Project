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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="BlueRight", group="Auto")
//@Disabled
public class BT_AutoBlueRight extends BT_AutoSuper {
    // Declare OpMode members.
    @Override
    public void initAutoConstants(){
        IS_2_CUBES = false;
        LEFT_DRIVE_DIST = 78.5;
        CENTER_DRIVE_DIST = 100;
        RIGHT_DRIVE_DIST = 119.5;
        CRYPTO_DIST = 22;
        CRYPTO_TURN = -90;
        CLOSE_CRYPTO_ANGLE = 0;
        SIDE_CRYPTO_ANGLE = -90;
        FINAL_ROBOT_ANGLE = 90;
        ALLIANCE_COLOR = "BLUE";
        TARGET_JEWEL_COLOR = BT_Jewels.JewelColor.RED;
    }

    @Override
    public void driveToCrypto(double driveDist){
        robot.drive.move(driveDist, BT_MecanumDrive.DriveDirection.BACKWARD, 2500 , telemetry );
    }

    @Override
    public void put2Cubes() {
        robot.intake.glyphsIn();
        robot.drive.move(68, BT_MecanumDrive.DriveDirection.FORWARD, 2500, telemetry, 0.5);
        robot.drive.move(90, BT_MecanumDrive.DriveDirection.FORWARD, 2500, telemetry, 0.3);
        robot.drive.turn(90, 2500, telemetry, true);
        robot.drive.move(140, BT_MecanumDrive.DriveDirection.BACKWARD, 2500, telemetry, 1);
        robot.intake.stop();
        robot.glyphs.autoArmMid();
        sleep(200);
        robot.drive.turn(90, 2500, telemetry, true);
        sleep(100);
        robot.drive.move(CRYPTO_DIST+10, BT_MecanumDrive.DriveDirection.BACKWARD, 2500, telemetry);
        robot.glyphs.releaseGlyphs();
        sleep(700);
        robot.drive.move(5, BT_MecanumDrive.DriveDirection.FORWARD, 2500, telemetry, 0.1);
        robot.drive.move(20, BT_MecanumDrive.DriveDirection.FORWARD, 2500, telemetry, 0.9);
        robot.glyphs.armDown(true);
        robot.drive.move(20, BT_MecanumDrive.DriveDirection.BACKWARD, 2500, telemetry);
    }
}

