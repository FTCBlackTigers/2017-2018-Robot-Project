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
import com.sun.tools.javac.util.Context;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.BT_FieldSetup;
import org.firstinspires.ftc.teamcode.BT_Hardware;
import org.firstinspires.ftc.teamcode.BT_Jewels;
import org.firstinspires.ftc.teamcode.BT_MecanumDrive;
import org.firstinspires.ftc.teamcode.BT_Status;
import org.firstinspires.ftc.teamcode.BT_Vumark;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;


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
@Autonomous(name="BT_CalibrateMecanum", group="Test")

public class BT_CalibrateMecanum extends LinearOpMode {
    // Declare OpMode members.
    protected ElapsedTime runtime = new ElapsedTime();
    protected BT_Hardware robot = new BT_Hardware();

    @Override
    public void runOpMode() {
        double num[] = {0,0,0,0};
        double maxPower =0;
        robot.init(hardwareMap,this);
        runtime.reset();
        telemetry.setAutoClear(false);

        waitForStart();



        robot.drive.frontLeftDrive.setPower(0.5);
        sleep(3000);
        robot.drive.frontLeftDrive.setPower(0);
        telemetry.addLine("frontLeftDrive" + robot.drive.frontLeftDrive.getCurrentPosition());
        telemetry.update();
        num[0] = robot.drive.frontLeftDrive.getCurrentPosition();

        robot.drive.rearLeftDrive.setPower(0.5);
        sleep(3000);
        robot.drive.rearLeftDrive.setPower(0);
        telemetry.addLine("rearLeftDrive" + robot.drive.rearLeftDrive.getCurrentPosition());
        telemetry.update();
        num[1]= robot.drive.rearLeftDrive.getCurrentPosition();



        robot.drive.frontRightDrive.setPower(0.5);
        sleep(3000);
        robot.drive.frontRightDrive.setPower(0);
        telemetry.addLine("frontRightDrive" + robot.drive.frontRightDrive.getCurrentPosition());
        telemetry.update();
        num[2] = robot.drive.frontRightDrive.getCurrentPosition();

        robot.drive.rearRightDrive.setPower(0.5);
        sleep(3000);
        robot.drive.rearRightDrive.setPower(0);
        telemetry.addLine("rearRighttDrive" + robot.drive.rearRightDrive.getCurrentPosition());
        telemetry.update();
        num[3] = robot.drive.rearRightDrive.getCurrentPosition();

        for (int i = 0; i<4; i++) {
            if (maxPower < num[i]) {
                maxPower = num[i];
            }
        }
        for (int i = 0; i<4; i++){
            num[i] = maxPower/ num[i];
        }

        try {
            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(hardwareMap.appContext.openFileOutput("DriveConfig.txt", android.content.Context.MODE_PRIVATE));
            for (int i = 0; i<4; i++) {
                outputStreamWriter.write(Double.toString(num[i])+"\n");
            }
            outputStreamWriter.close();
        } catch (java.io.IOException e) {
            telemetry.addLine("file write failed");
            telemetry.update();

        }

        //        try {
//            InputStreamReader inputStreamReader = new InputStreamReader(hardwareMap.appContext.openFileInput("DriveConfig.txt"));
//            BufferedReader bufferedReader = new BufferedReader(inputStreamReader);
//            for(int i = 0; i<4; i++) {
//                StringBuilder stringBuilder = new StringBuilder();
//                stringBuilder.append(bufferedReader.readLine());
//                num[i] = Integer.parseInt(stringBuilder.toString());
//            }
//        } catch (java.io.IOException e) {
//            telemetry.addLine("file not found");
//            telemetry.update();
//        }

        telemetry.addData("drives","%f, %f, %f, %f",num[0],num[1],num[2],num[3]);
        telemetry.update();
        sleep(20000);
    }
}
