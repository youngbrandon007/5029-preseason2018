package org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleConfigLinearOpMode;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleMotor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleRobot;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleServo;

/**
 * Created by young on 9/14/2017.
 */

abstract public class RelicRecoveryConfigPrototypeV2 extends PineappleConfigLinearOpMode {

    public PineappleMotor driveRight;
    public PineappleMotor driveLeft;
    public PineappleMotor lift;
    public PineappleMotor collector;
    public PineappleServo flip;
    public PineappleServo compWheelRight;
    public PineappleServo compWheelLeft;

    public static final double FLIPDOWN = 0.05;
    public static final double FLIPFLAT = 0.20;
    public static final double FLIPUP = 0.60;

    @Override
    public void config(LinearOpMode linearOpMode) {
        robotHandler = new PineappleRobot(linearOpMode);


        driveRight = robotHandler.motorHandler.newDriveMotor("R", 1, false, false, PineappleEnum.MotorLoc.RIGHTFRONT, PineappleEnum.MotorType.NEV40);
        driveLeft = robotHandler.motorHandler.newDriveMotor("L", 1, false, false, PineappleEnum.MotorLoc.LEFTFRONT, PineappleEnum.MotorType.NEV40);
        collector = robotHandler.motorHandler.newDriveMotor("C", 1, false, false, PineappleEnum.MotorLoc.RIGHTFRONT, PineappleEnum.MotorType.NEV40);
        lift = robotHandler.motorHandler.newDriveMotor("lift", 1, false, false, PineappleEnum.MotorLoc.LEFTFRONT, PineappleEnum.MotorType.NEV40);
        flip = robotHandler.servoHandler.newLimitServo("flip", 1, 0);
        compWheelLeft = robotHandler.servoHandler.newContinuousServo("WL", 0.5);
        compWheelRight = robotHandler.servoHandler.newContinuousServo("WR", 0.5);
        telemetry.update();

    }



}
