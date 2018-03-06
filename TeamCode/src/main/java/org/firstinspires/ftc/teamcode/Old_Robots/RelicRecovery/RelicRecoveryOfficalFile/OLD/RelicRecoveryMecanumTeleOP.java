package org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.OLD;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources.RelicRecoveryConfig;

/**
 * Created by Brandon on 10/10/2017.
 */
@TeleOp(name = "RRmecanum", group = "Linear Opmode")
@Disabled
public class
RelicRecoveryMecanumTeleOP extends RelicRecoveryConfig {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        config(this);

        waitForStart();
        runtime.reset();

        while(opModeIsActive()){
            robotHandler.drive.mecanum.updateMecanum(gamepad1, 1);
        }



    }
}