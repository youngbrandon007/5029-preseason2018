package org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFileAfterWV.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFileAfterWV.RelicRecoveryResources.RelicRecoveryConfigV2Cleve;

/**
 * Created by ftcpi on 11/27/2017.
 */
@Autonomous(name = "SwitchBoardSettingCleve")
@Disabled
public class RelicRecoverySwitchBoardSettingCleve extends RelicRecoveryConfigV2Cleve {
    @Override
    public void runOpMode() throws InterruptedException {
        config(this);
        waitForStart();
        loadSwitchBoardLoop();
    }
}
