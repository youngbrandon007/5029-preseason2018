package org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.Auto.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources.RelicRecoveryConfigV2;

import java.text.DecimalFormat;

/**
 * Created by ftcpi on 11/16/2017.
 */
@Autonomous(name = "PID", group = "TUning")
@Disabled
public class RelicRecoveryPIDTuning extends RelicRecoveryConfigV2 {
    @Override
    public void runOpMode() throws InterruptedException {


        DecimalFormat decimalFormat = new DecimalFormat("#.########");
        config(this);
        AnalogInput PPot = hardwareMap.analogInput.get("P");
        AnalogInput IPot = hardwareMap.analogInput.get("delay");
        AnalogInput DPot = hardwareMap.analogInput.get("D");

        double P = Double.parseDouble(decimalFormat.format(PPot.getVoltage()/20));
        double I = Double.parseDouble(decimalFormat.format(IPot.getVoltage()/50));

        double D = Double.parseDouble(decimalFormat.format(DPot.getVoltage()/50));
        while (!opModeIsActive()) {
            telemetry.addData("P:", P);
            telemetry.addData("I:", I*1000);
            telemetry.addData("D:", D);
            PPot.getVoltage();
            P = Double.parseDouble(decimalFormat.format(PPot.getVoltage()/20));
            I = Double.parseDouble(decimalFormat.format(IPot.getVoltage()/100));
            D = Double.parseDouble(decimalFormat.format(DPot.getVoltage()/20));
            telemetry.update();
        }
        telemetry.addData("PID", "Tuning Finished Press play");
        telemetry.update();
        waitForStart();
        robotHandler.auto.gyroTurnPID(90, P, I, D, navx_device);

    }
}
