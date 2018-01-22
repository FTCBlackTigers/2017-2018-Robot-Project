package Samples;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Black Tigers on 05/12/2017.
 */
@Autonomous(name = "Concept: Scan Servo", group = "Concept")
@Disabled
public class JewelArm extends LinearOpMode{

        NormalizedColorSensor colorSensor;

        @Override
        public void runOpMode() {

            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

            // Connect to servo (Assume PushBot Left Hand)
            // Change the text in quotes to match any servo name on your robot.
            //armServo.setPosition(0.65);
            // Wait for the start button
            telemetry.addData(">", "Press Start to scan Servo." );
            telemetry.update();
            waitForStart();

            // switch licht off

            // read color valueand display
            while(opModeIsActive()){
                NormalizedRGBA colors = colorSensor.getNormalizedColors();
                int color = colors.toColor();
                telemetry.addLine("raw Android color: ")
                        .addData("a", "%02x", Color.alpha(color))
                        .addData("r", "%02x", Color.red(color))
                        .addData("g", "%02x", Color.green(color))
                        .addData("b", "%02x", Color.blue(color));
                telemetry.update();

                // slew the servo, according to the rampUp (direction) variable.
            }

            // Signal done;
            telemetry.addData(">", "Done");
            telemetry.update();
        }
}
