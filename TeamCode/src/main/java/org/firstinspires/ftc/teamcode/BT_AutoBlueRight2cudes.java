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

import android.os.AsyncTask;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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

@Autonomous(name="BlueRight2cubes", group="Auto")
//@Disabled
public class BT_AutoBlueRight2cudes extends BT_AutoSuper {
    // Declare OpMode members.
    @Override
    public void initAutoConstants() {
        IS_2_CUBES = true;
        LEFT_DRIVE_DIST = 76;
        CENTER_DRIVE_DIST = 101;
        RIGHT_DRIVE_DIST = 117;
        CRYPTO_DIST = 23;
        CLOSE_CRYPTO_ANGLE = 0;
        SIDE_CRYPTO_ANGLE = -90;
        FINAL_ROBOT_ANGLE = 90;
        TARGET_JEWEL_COLOR = BT_Jewels.JewelColor.RED;
    }

    @Override
    public void driveToCrypto(double driveDist) {
        robot.drive.move(driveDist, BT_MecanumDrive.DriveDirection.BACKWARD, 2500, telemetry);


    }

    @Override
    public void put2Cubes() {
        robot.drive.turn(90, 3000, telemetry, true);
        robot.intake.glyphsIn();
        robot.drive.move(120, BT_MecanumDrive.DriveDirection.FORWARD, 2500, telemetry);
        robot.drive.move(100, BT_MecanumDrive.DriveDirection.BACKWARD, 2500, telemetry);
        robot.intake.stop();
        robot.glyphs.catchGlyphs();
        sleep(500);
        robot.glyphs.autoArmHigh();
        sleep(250);
        robot.glyphs.armLow();
        robot.drive.turn(90, 2500, telemetry, true);
        sleep(1000);
        robot.drive.move(35, BT_MecanumDrive.DriveDirection.BACKWARD, 2500, telemetry);
        robot.glyphs.releaseGlyphs();
        sleep(250);
        robot.drive.move(20, BT_MecanumDrive.DriveDirection.FORWARD, 2500, telemetry);
        robot.glyphs.armDown(true);
        sleep(250);
        robot.drive.move(20, BT_MecanumDrive.DriveDirection.BACKWARD, 2500, telemetry);
        robot.drive.move(10, BT_MecanumDrive.DriveDirection.FORWARD, 2500, telemetry);


    }

}