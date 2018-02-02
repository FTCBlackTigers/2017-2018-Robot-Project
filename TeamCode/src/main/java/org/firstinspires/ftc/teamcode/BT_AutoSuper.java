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

public class BT_AutoSuper extends LinearOpMode {
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
        initAutoConstants();
        BT_FieldSetup.closeCryptobox = CLOSE_CRYPTO_ANGLE;
        BT_FieldSetup.sideCryptobox = SIDE_CRYPTO_ANGLE;

        double driveDist = 0 ;
        robot.init(hardwareMap,this);
        BT_Status.cleanStatus();
        BT_Status.addLine("Robot Initialized");
        telemetry.addData("Status", BT_Status.getStatusLine());
        telemetry.update();

        BT_Vumark btVumark = new BT_Vumark(hardwareMap) ;
        BT_Status.addLine("Vumark Initialized");
        telemetry.addData("Status",BT_Status.getStatusLine());
        telemetry.update();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN ;
        // Wait for the game to start (driver presses PLAY)
        BT_Status.addLine("Waiting for start...");
        telemetry.addData("Status",BT_Status.getStatusLine());
        telemetry.update();
        waitForStart();
        BT_Status.addLine("Started...");
        telemetry.addData("Status",BT_Status.getStatusLine());
        telemetry.update();
        runtime.reset();
        while ((vuMark == RelicRecoveryVuMark.UNKNOWN) && (runtime.milliseconds()< WAIT_FOR_VUMARK)) {
            vuMark = btVumark.getVuMark();
        }
        switch (vuMark) {
            case RIGHT :
                driveDist = RIGHT_DRIVE_DIST ;
                break;
            case CENTER:
                driveDist = CENTER_DRIVE_DIST ;
                break;
            case LEFT:
                driveDist = LEFT_DRIVE_DIST ;
                break;
            case UNKNOWN:
                driveDist = RIGHT_DRIVE_DIST ;
                break;
        }

        robot.jewels.moveJewel(TARGET_JEWEL_COLOR);
        telemetry.addLine(BT_Status.getStatusLine());
        telemetry.addData("Status", "Identified column: %s ",vuMark);
        telemetry.addData("dist", driveDist);
        telemetry.setAutoClear(false);
        telemetry.update();
        sleep(500);
        driveToCrypto(driveDist);
        robot.intake.glyphsOut();
        sleep(500);
        robot.drive.move(5, BT_MecanumDrive.DriveDirection.BACKWARD, 2500 , telemetry );
        sleep(500);
        robot.intake.stop();
        robot.drive.move(20, BT_MecanumDrive.DriveDirection.BACKWARD, 2500 , telemetry );
        robot.drive.turn(FINAL_ROBOT_ANGLE,3000,telemetry, true);
        robot.drive.move(30,  BT_MecanumDrive.DriveDirection.BACKWARD, 2500, telemetry);
        robot.drive.move(20,  BT_MecanumDrive.DriveDirection.FORWARD, 2500, telemetry);


//        robot.drive.turn(180,3000,telemetry);
//        robot.drive.move(40, BT_MecanumDrive.DriveDirection.BACKWARD, 2500 , telemetry );
//        robot.intake.glyphsIn();
//        robot.drive.move(40, BT_MecanumDrive.DriveDirection.FORWARD, 1000 , telemetry );
//        robot.glyphs.armHigh();
//        robot.glyphs.releaseGlyphs();

    }
    public void driveToCrypto(double driveDist){
    }

}
