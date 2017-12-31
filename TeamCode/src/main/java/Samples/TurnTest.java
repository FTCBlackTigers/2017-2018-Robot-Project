package Samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BT_TankDrive;

/**
 * Created by Black Tigers on 09/12/2017.
 */

@Autonomous(name="turnTest", group="Auto")
@Disabled
public class TurnTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private BT_TankDrive robot = new BT_TankDrive();
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
