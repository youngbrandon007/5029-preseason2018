package org.firstinspires.ftc.teamcode.RelicRecoveryWorlds;

import android.os.Environment;
import android.util.Log;

import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources.RelicRecoveryConstants;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.Calendar;

/**
 * Created by ftcpi on 2/7/2018.
 */
@Autonomous(name = "PID")
public class WorldPIDTesting extends WorldConfig {

    AnalogInput PPot;
    AnalogInput IPot;
    AnalogInput DPot;

    DecimalFormat decimalFormat;

    double P;
    double I;
    double D;

    navXPIDController.PIDResult yawPIDResult;

    String data;
    String name;

    final double TARGET_ANGLE_DEGREES = 90;
    final double TOLERANCE_DEGREES = 2.0;

    final int DEVICE_TIMEOUT_MS = 1000;

    ElapsedTime el = new ElapsedTime();

    @Override
    public void init() {

        decimalFormat = new DecimalFormat("#.#####");

        config(this);

        while (!calibration_complete) {
            calibration_complete = !navx_device.isCalibrating();

        }
        navx_device.zeroYaw();

        PPot = hardwareMap.analogInput.get("P");
        IPot = hardwareMap.analogInput.get("I");
        DPot = hardwareMap.analogInput.get("D");
    }

    @Override
    public void init_loop() {

        P = Double.parseDouble(decimalFormat.format(PPot.getVoltage() / 50));
        I = Double.parseDouble(decimalFormat.format(IPot.getVoltage() / 100));
        D = Double.parseDouble(decimalFormat.format(DPot.getVoltage() / 50));

        telemetry.addData("P:", P);
        telemetry.addData("I*1000:", I * 1000);
        telemetry.addData("D:", D);
        PPot.getVoltage();
//      P = Double.parseDouble(decimalFormat.format(PPot.getVoltage()/50));
        P = WorldConstants.PID.P;
//      I = Double.parseDouble(decimalFormat.format(IPot.getVoltage()/100));
//      D = Double.parseDouble(decimalFormat.format(DPot.getVoltage()/50));
        I = WorldConstants.PID.I;
        D = WorldConstants.PID.D;
        telemetry.update();

        telemetry.addData("PID", "Tuning Finished Press play");
        telemetry.update();
    }

    @Override
    public void start() {

        Calendar calendar = Calendar.getInstance();
        int year = calendar.get(Calendar.YEAR);
        int month = calendar.get(Calendar.MONTH);
        int day = calendar.get(Calendar.DATE);
        int hour = calendar.get(Calendar.HOUR_OF_DAY);
        int minute = calendar.get(Calendar.MINUTE);
        int second = calendar.get(Calendar.SECOND);
        int millisecond = calendar.get(Calendar.MILLISECOND);
        String full = "" + year + month + day + "_" + hour + minute + second + millisecond;
        name = full + "_" + TARGET_ANGLE_DEGREES + "_ " + P + "_" + I + "_" + D + "_";
        data = "TIME,OUTPUT,YAW,TARGET";

        double YAW_PID_P = P;
        double YAW_PID_I = I;
        double YAW_PID_D = D;
        navXPIDController yawPIDController;
        yawPIDController = new navXPIDController(navx_device, navXPIDController.navXTimestampedDataSource.YAW);

		/* Configure the PID controller */
        yawPIDController.enable(true);
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(false);
        yawPIDController.setOutputRange(RelicRecoveryConstants.MIN_MOTOR_OUTPUT_VALUE, RelicRecoveryConstants.MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);

        yawPIDResult = new navXPIDController.PIDResult();

        el.reset();
    }

    @Override
    public void loop() {
        try {
                double output = 0;
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        telemetry.addData("PIDOutput", decimalFormat.format(0.00));
                    } else {
                        output = yawPIDResult.getOutput();
                        robotHandler.drive.tank.setPower(output, output);

                        telemetry.addData("PIDOutput", decimalFormat.format(output) + ", " + decimalFormat.format(-output));
                    }
                } else {
                /* A timeout occurred */
                    telemetry.addData("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
                telemetry.addData("Yaw", decimalFormat.format(navx_device.getYaw()));

                data += "\n" + el.milliseconds() + "," + output * 100 + "," + navx_device.getYaw() + "," + TARGET_ANGLE_DEGREES;

                telemetry.update();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }


    }

    @Override
    public void stop() {
        navx_device.close();
        writeToFile(name, data);
    }

    private void writeToFile(String name, String data) {
        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
        File file = new File(path + "/PID", name + ".csv");
        try {
            FileOutputStream stream = new FileOutputStream(file, true);
            stream.write(data.getBytes());
            stream.close();
            Log.i("saveData", "Data Saved");
        } catch (IOException e) {
            Log.e("SAVE DATA", "Could not write file " + e.getMessage());
        }
    }
}
