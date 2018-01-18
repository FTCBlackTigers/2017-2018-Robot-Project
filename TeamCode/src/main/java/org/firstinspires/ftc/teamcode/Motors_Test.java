package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Black Tigers on 16/01/2018.
 */
@TeleOp(name="Motors_Test", group="Teleop")
public class Motors_Test extends OpMode {
    private BT_Hardware robot = new BT_Hardware();

    @Override
    public void init() {
        robot.init(hardwareMap, this);
        telemetry.addData("Status", "Robot Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        buttons(gamepad1,telemetry);
    }
    public void buttons (Gamepad gamepad, Telemetry telemetry) {
        double speed = 1;
        robot.drive.buttonDrive(gamepad, telemetry);
        if (gamepad.right_bumper) {
            speed = -1;
        }
        if (gamepad.left_trigger > 0.5) {
            robot.intake.leftIntake.setPower(speed);
        } else {
            robot.intake.leftIntake.setPower(0);
        }
        if (gamepad.right_trigger > 0.5) {
            robot.intake.rightIntake.setPower(speed);
        } else {
            robot.intake.rightIntake.setPower(0);
        } if (gamepad.left_bumper) {
            robot.intake.intakeMotor.setPower(speed);
        } else {
            robot.intake.intakeMotor.setPower(0);
        }
    }
}
