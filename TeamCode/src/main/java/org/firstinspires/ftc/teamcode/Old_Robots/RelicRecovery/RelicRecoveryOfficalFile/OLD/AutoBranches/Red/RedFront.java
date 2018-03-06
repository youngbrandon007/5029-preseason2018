package org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.OLD.AutoBranches.Red;

import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.Auto.RelicRecoveryAbstractAutonomous;

/**
 * Created by Brandon on 11/14/2017.
 */

public class RedFront extends RelicRecoveryAbstractAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("OpMode", "Red");
        telemetry.update();
        waitForStart();

    }
}
