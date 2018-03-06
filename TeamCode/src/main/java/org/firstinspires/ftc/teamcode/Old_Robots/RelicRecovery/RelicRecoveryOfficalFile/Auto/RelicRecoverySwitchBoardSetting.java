package org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by ftcpi on 11/27/2017.
 */
@Autonomous(name = "SwitchBoardSetting")
@Disabled
public class RelicRecoverySwitchBoardSetting extends RelicRecoveryAbstractAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        config(this);
        waitForStart();
        loadSwitchBoardLoop();
    }
}
