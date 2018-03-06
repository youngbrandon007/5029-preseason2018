package org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleConfigLinearOpMode;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleMotor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleRobot;

/**
 * Created by young on 9/14/2017.
 */

abstract public class RelicRecoveryConfigPrototype extends PineappleConfigLinearOpMode {

    public PineappleMotor driveRight;
    public PineappleMotor driveLeft;
    public PineappleMotor collectRight;
    public PineappleMotor collectLeft;




    @Override
    public void config(LinearOpMode linearOpMode) {
        robotHandler = new PineappleRobot(linearOpMode);


        driveLeft = robotHandler.motorHandler.newDriveMotor("L", 1, false, false, PineappleEnum.MotorLoc.LEFT, PineappleEnum.MotorType.NEV40);
        driveRight = robotHandler.motorHandler.newDriveMotor("R", 1, false, false, PineappleEnum.MotorLoc.RIGHT, PineappleEnum.MotorType.NEV40);
        collectRight = robotHandler.motorHandler.newDriveMotor("CL", 1, false, false, PineappleEnum.MotorLoc.NONE, PineappleEnum.MotorType.NEV40);
        collectLeft = robotHandler.motorHandler.newDriveMotor("CR", 1, false, false, PineappleEnum.MotorLoc.NONE, PineappleEnum.MotorType.NEV40);

        //gyroSensor = robotHandler.sensorHandler.newGyroSensor("gyro");



    }

}
