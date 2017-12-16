package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Black Tigers on 09/12/2017.
 */

@Autonomous(name="turnTest", group="Auto")
public class TurnTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private BT_Drive robot = new BT_Drive();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);

        waitForStart();
        runtime.reset();
    if (opModeIsActive()) {
        robot.turn(90, 10000,telemetry);
    }

    }


}
