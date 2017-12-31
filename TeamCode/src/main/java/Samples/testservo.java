package Samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Black Tigers on 29/12/2017.
 */

@TeleOp(name="servoTest", group="")
public class testservo extends OpMode {

    Servo servo;
    double position=0.4;
    @Override
    public void init() {
        servo=hardwareMap.get(Servo.class,"glyphsArm");
        servo.setPosition(0.2);
    }

    @Override
    public void loop() {
        boolean up = gamepad1.y;
        boolean down = gamepad1.a;

        if(up){
            servo.setPosition(0.7);
        }
        else if(down)
        {
            servo.setPosition(0.2);
        }
        else if(gamepad1.left_bumper){
            servo.setPosition(0.9);
        }

//        if(up) {
//            if (position < 0.8) {
////                position = 0.8;
//                position += 0.1;
//            }
//            telemetry.addData("incresment", position);
////            servo.setPosition(position);
//        }
//        else if(down) {
//
//
//            if (position > 0.2) {
////                position = 0.2;
//                position -= 0.1;
//            }
//            telemetry.addData("incresment", position);
////            servo.setPosition(position);
//        }
//        servo.setPosition(position);

        telemetry.addData("position", servo.getPosition());
        telemetry.update();
    }
}
