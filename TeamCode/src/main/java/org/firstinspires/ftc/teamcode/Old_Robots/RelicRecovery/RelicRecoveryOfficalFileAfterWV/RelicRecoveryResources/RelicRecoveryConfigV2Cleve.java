package org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFileAfterWV.RelicRecoveryResources;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleConfigLinearOpMode;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleMotor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleRobot;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleServo;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors.PineappleTouchSensor;
import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources.RelicRecoveryConstants;
import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources.RelicRecoveryEnums;

/**
 * Created by Brandon on 12/5/2017.
 */

public abstract class RelicRecoveryConfigV2Cleve extends PineappleConfigLinearOpMode{


    //Robot
    //Motors
    public PineappleMotor driveFrontRight;
    public PineappleMotor driveFrontLeft;
    public PineappleMotor driveBackRight;
    public PineappleMotor driveBackLeft;
    public PineappleMotor conveyLeft;
    public  PineappleMotor conveyRight;
    //Servos
    public PineappleServo collectorLeft;
    public PineappleServo collectorRight;
    public PineappleServo releaseLeft;
    public PineappleServo releaseRight;
    public PineappleServo jewel;
    public PineappleServo alignLeft;
    public PineappleServo alignRight;
    public PineappleTouchSensor cryptoTouchSensor;

    public final int NAVX_DIM_I2C_PORT = 0;
    public AHRS navx_device;
    public final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    //switch Board
    public double delay = 0;
    public RelicRecoveryEnums.AutoColor color = RelicRecoveryEnums.AutoColor.BLUE;
    public RelicRecoveryEnums.StartingPosition position = RelicRecoveryEnums.StartingPosition.FRONT;
    public RelicRecoveryEnums.ColorPosition colorPosition = RelicRecoveryEnums.ColorPosition.BLUEFRONT;
    public boolean moreGlyph = false;
    public boolean glyphsEnabled = true;
    public boolean delayEnabled = true;
    public boolean pidEnabled = false;
    public boolean jewelsEnabled = true;
    private boolean usingGyro = false;
    public ElapsedTime wait = new ElapsedTime();

    public boolean calibration_complete = false;
    public PineappleEnum.JewelState state = PineappleEnum.JewelState.NON_NON;
    public double turnCount = 0;


    @Override
    public void config(LinearOpMode linearOpMode) {
        robotHandler = new PineappleRobot(linearOpMode);

        driveFrontRight = robotHandler.motorHandler.newDriveMotor("FR", 1, false, false, PineappleEnum.MotorLoc.RIGHTFRONT, PineappleEnum.MotorType.NEV40);
        driveFrontLeft = robotHandler.motorHandler.newDriveMotor("FL", 1, false, false, PineappleEnum.MotorLoc.LEFTFRONT, PineappleEnum.MotorType.NEV40);
        driveBackRight = robotHandler.motorHandler.newDriveMotor("BR", 1, false, false, PineappleEnum.MotorLoc.RIGHTBACK, PineappleEnum.MotorType.NEV40);
        driveBackLeft = robotHandler.motorHandler.newDriveMotor("BL", 1, false, false, PineappleEnum.MotorLoc.LEFTBACK, PineappleEnum.MotorType.NEV40);
        conveyLeft = robotHandler.motorHandler.newDriveMotor("ConL", 1, false, false, PineappleEnum.MotorLoc.NONE, PineappleEnum.MotorType.NEV40);
        conveyRight = robotHandler.motorHandler.newDriveMotor("ConR", 1, false, false, PineappleEnum.MotorLoc.NONE, PineappleEnum.MotorType.NEV40);

        cryptoTouchSensor = robotHandler.sensorHandler.newTouchSensor("crypto");
        collectorLeft = robotHandler.servoHandler.newContinuousServo("CL", 0.5);
        collectorRight = robotHandler.servoHandler.newContinuousServo("CR", 0.5);
        releaseLeft = robotHandler.servoHandler.newLimitServo("CFL", 1, RelicRecoveryConstants.FLIPINLEFT);
        releaseRight = robotHandler.servoHandler.newLimitServo("CFR", 1, RelicRecoveryConstants.FLIPINRIGHT);
        jewel = robotHandler.servoHandler.newLimitServo("J", 1 , RelicRecoveryConstants.JEWELUP);
        alignLeft = robotHandler.servoHandler.newLimitServo("AL", 1, RelicRecoveryConstants.ALIGNUPLEFT);
        alignRight = robotHandler.servoHandler.newLimitServo("AR", 1, RelicRecoveryConstants.ALIGNUPRIGHT);
        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);
    }

    public void loadSwitchBoard() {
        delay = robotHandler.switchBoard.loadAnalog("delay") * 2;
        color = (robotHandler.switchBoard.loadDigital("color")) ? RelicRecoveryEnums.AutoColor.BLUE : RelicRecoveryEnums.AutoColor.RED;
        position = (robotHandler.switchBoard.loadDigital("position")) ? RelicRecoveryEnums.StartingPosition.FRONT : RelicRecoveryEnums.StartingPosition.BACK;
        jewelsEnabled = robotHandler.switchBoard.loadDigital("jewel");
        pidEnabled = robotHandler.switchBoard.loadDigital("pid");
        glyphsEnabled = robotHandler.switchBoard.loadDigital("glyph");
        delayEnabled = robotHandler.switchBoard.loadDigital("delayEnabled");
        delay = Math.round(delay * 2) / 2.0;

        switch (color){

            case RED:
                switch (position){

                    case FRONT:
                        colorPosition = RelicRecoveryEnums.ColorPosition.REDFRONT;
                        break;
                    case BACK:
                        colorPosition = RelicRecoveryEnums.ColorPosition.REDBACK;
                        break;
                }
                break;
            case BLUE:
                switch (position){

                    case FRONT:
                        colorPosition = RelicRecoveryEnums.ColorPosition.BLUEFRONT;
                        break;
                    case BACK:
                        colorPosition = RelicRecoveryEnums.ColorPosition.BLUEBACK;
                        break;
                }
                break;
        }
        telemetry.addData("Delay", delay);
        telemetry.addData("DelayEnabled", delayEnabled);
        telemetry.addData("Color", color);
        telemetry.addData("Jewel", jewelsEnabled);
        telemetry.addData("PID", pidEnabled);
        telemetry.addData("Glyphs", glyphsEnabled);
        telemetry.addData("Position", position);
    }

    public void loadSwitchBoardLoop() throws InterruptedException {
        while (opModeIsActive()) {
            loadSwitchBoard();
            telemetry.update();
            Thread.sleep(100);
        }
    }
}
