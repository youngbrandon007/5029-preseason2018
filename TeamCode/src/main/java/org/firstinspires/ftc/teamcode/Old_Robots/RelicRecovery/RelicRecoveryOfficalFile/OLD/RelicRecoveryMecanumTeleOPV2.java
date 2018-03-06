package org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.OLD;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources.RelicRecoveryConfigV2;

/**
 * Created by A Pineapple on 10/10/2017.
 */
@TeleOp(name = "RRmecanumV2", group = "Linear Opmode")
@Disabled
public class
RelicRecoveryMecanumTeleOPV2 extends RelicRecoveryConfigV2 {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        config(this);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            robotHandler.drive.mecanum.updateMecanum(gamepad1, 1);
            if (gamepad1.left_bumper) {
                collectorLeft.setPosition(1);
                conveyRight.setPower(-1);
                conveyLeft.setPower(1);
            } else if (gamepad1.right_bumper) {
                collectorLeft.setPosition(0);
                conveyRight.setPower(1);
                conveyLeft.setPower(-1);
            } else {
                collectorLeft.setPosition(0.5);
                conveyRight.setPower(0);
                conveyLeft.setPower(0);
            }

        }


    }
}

