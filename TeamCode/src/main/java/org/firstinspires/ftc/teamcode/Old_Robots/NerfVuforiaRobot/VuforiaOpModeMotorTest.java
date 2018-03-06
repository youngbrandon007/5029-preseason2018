
package org.firstinspires.ftc.teamcode.Old_Robots.NerfVuforiaRobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Brandon on 3/31/2017.
 */

@TeleOp(name = "VuforiaMotorTest", group = "Linear Opmode")
@Disabled

public class VuforiaOpModeMotorTest extends com.qualcomm.robotcore.eventloop.opmode.LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor Rotate;
    DcMotor Tilt;

    Servo Trigger;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Rotate = hardwareMap.dcMotor.get("r");
        Tilt = hardwareMap.dcMotor.get("t");

        Trigger = hardwareMap.servo.get("tr");

        Trigger.setPosition(1);

        waitForStart();

        Trigger.setPosition(.0);

        Rotate = hardwareMap.dcMotor.get("r");

        Rotate.setPower(.1);
        Tilt.setPower(.1);


        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run : " + runtime.toString());
            telemetry.update();





        }
    }

}

