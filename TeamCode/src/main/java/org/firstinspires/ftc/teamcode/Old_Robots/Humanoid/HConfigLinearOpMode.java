package org.firstinspires.ftc.teamcode.Old_Robots.Humanoid;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleConfigLinearOpMode;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleMotor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleRobot;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleRobotConstants;

/**
 * Created by Brandon on 8/26/2017.
 */

public class HConfigLinearOpMode extends PineappleConfigLinearOpMode {

    PineappleRobot robot;

//    Servo HandFourFingers;
//    Servo Head;
    PineappleMotor rShoulder;
    PineappleMotor lShoulder;
    //    PineappleMotor Elbow;
    PineappleMotor RightDrive;
    PineappleMotor LeftDrive;
    @Override
    public void config(LinearOpMode linearOpMode) {
        robot = new PineappleRobot(linearOpMode);

//        HandFourFingers = linearOpMode.hardwareMap.servo.get("HFF");
//        Head = linearOpMode.hardwareMap.servo.get("H");
        RightDrive = robot.motorHandler.newDriveMotor("RD", PineappleEnum.MotorLoc.RIGHT, PineappleEnum.MotorType.NEV60);
        LeftDrive = robot.motorHandler.newDriveMotor("LD", PineappleEnum.MotorLoc.LEFT, PineappleEnum.MotorType.NEV60);
        RightDrive.maxPower = 0.5;
        LeftDrive.maxPower = 0.5;
        rShoulder = robot.motorHandler.newMotor("S", 1,false, false, PineappleEnum.MotorType.NEV60);
//        Elbow = robot.motorHandler.newMotor("E", 1,false, false, PineappleEnum.MotorType.NEV60);
//        Elbow.maxPower = 0.1;

//        setHandRestingPos();
    }

//    /**
//     * This method sets the position
//     * @param val
//     */
//    public void setFourFingersPos(double val){
//        HandFourFingers.setPosition(val);
//    }
//    public void setHandRestingPos(){
//        setFourFingersPos(PineappleRobotConstants.HUMANOIDFINGERREST);
//    }
//    public void setHandFullPos(){
//        setFourFingersPos(PineappleRobotConstants.HUMANOIDFINGERFULL);
//    }
//    @Override
    public void runOpMode() throws InterruptedException {

}
}
