package org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.OLD;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources.RelicRecoveryConfigPrototype;

/**
 * Created by young on 9/14/2017.
 */
@TeleOp(name = "RelicRecoveryTeleOpPrototype", group = "Linear Opmode")
@Disabled
public class RelicRecoveryTeleOpPrototype extends RelicRecoveryConfigPrototype {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        config(this);

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
//            robotHandler.drive.setDirectPower(gamepad1.left_stick_y, gamepad1.right_stick_y);
            if (gamepad1.right_bumper) {
                collectLeft.update(1);
                collectRight.update(-1);
            } else if (gamepad1.left_bumper) {
                collectLeft.update(-1);
                collectRight.update(1);
            } else {
                collectLeft.update(0);
                collectRight.update(0);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }

    }
}
