package org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleRobot;

/**
 * Created by Brandon on 11/8/2017.
 */

public class RelicRecoveryVuforia {
    public static void alignWithGyro(VuforiaTrackableDefaultListener listener, VectorF vector, PineappleRobot robotHandler) throws InterruptedException {

        boolean go = true;

        while(go && robotHandler.opModeIsActive()){
            if(null != listener.getPose()){

                double robotAngle = getRobotAngle(listener) - 45;
                robotAngle += (robotAngle< 0) ? 360: 0;

                double distance = getDistance(listener, vector);
                double moveAngle = getMoveAngle(listener, vector)+90;
                moveAngle -=(moveAngle>360) ? 360:0;
                double rotation;

                robotHandler.sayFeedBack("Angle°", robotAngle);
                robotHandler.sayFeedBack("Distance", distance);
                robotHandler.sayFeedBack("Drive°", moveAngle);
                robotHandler.updateFeedBack();
                //Robot Rotation First
                if (robotAngle > 5 && robotAngle < 180) {
                    //Put Gyro here
//                    rotation = .1;
//                    robotHandler.drive.tank.setPower(rotation, rotation);
                } else if (robotAngle < 355 && robotAngle > 179) {
                    //and here
//                    rotation = -.1;
//                    robotHandler.drive.tank.setPower(rotation, rotation);
//                    Thread.sleep(100);
//                    robotHandler.drive.stop();
//                    Thread.sleep(50);
                }else{
                    //Robot Movement
                    if(distance > 50) {
                        robotHandler.drive.mecanum.setMecanum(moveAngle, .5, 0, 1);
                    }else if(distance > 5){
                        robotHandler.drive.mecanum.setMecanum(moveAngle, .3, 0, 1);
                        Thread.sleep(100);
                        robotHandler.drive.stop();
                        Thread.sleep(50);
                    }else{
                        go = false;
                    }
                }
            }else{

            }
        }
        robotHandler.drive.stop();
    }


    public static void align(VuforiaTrackableDefaultListener listener, VectorF vector, PineappleRobot robotHandler) throws InterruptedException {

        boolean go = true;

        while(go && robotHandler.opModeIsActive()){
            if(null != listener.getPose()){

                double robotAngle = getRobotAngle(listener) - 45;
                robotAngle += (robotAngle< 0) ? 360: 0;

                double distance = getDistance(listener, vector);
                double moveAngle = getMoveAngle(listener, vector)+90;
                moveAngle -=(moveAngle>360) ? 360:0;
                double rotation;

                robotHandler.sayFeedBack("Angle°", robotAngle);
                robotHandler.sayFeedBack("Distance", distance);
                robotHandler.sayFeedBack("Drive°", moveAngle);
                robotHandler.updateFeedBack();
                //Robot Rotation First
                if (robotAngle > 5 && robotAngle < 180) {
                    rotation = .1;
                    robotHandler.drive.tank.setPower(rotation, rotation);
                } else if (robotAngle < 355 && robotAngle > 179) {
                    rotation = -.1;
                    robotHandler.drive.tank.setPower(rotation, rotation);
                    Thread.sleep(100);
                    robotHandler.drive.stop();
                    Thread.sleep(50);
                }else{
                    //Robot Movement
                    if(distance > 50) {
                        robotHandler.drive.mecanum.setMecanum(moveAngle, .5, 0, 1);
                    }else if(distance > 5){
                        robotHandler.drive.mecanum.setMecanum(moveAngle, .3, 0, 1);
                        Thread.sleep(100);
                        robotHandler.drive.stop();
                        Thread.sleep(50);
                    }else{
                        go = false;
                    }
                }
            }else{
            }
        }
        robotHandler.drive.stop();
    }

    public static double getRobotAngle(VuforiaTrackableDefaultListener listener){
        VectorF angles = anglesFromTarget(listener);
        double robotAngle = Math.toDegrees(angles.get(0)) + 180;
        return robotAngle;
    }

    public static double getDistance(VuforiaTrackableDefaultListener listener, VectorF vector){
        VectorF angles = anglesFromTarget(listener);

        VectorF tran = navOffWall(listener.getPose().getTranslation(), Math.toDegrees(angles.get(0)), vector);

        return Math.sqrt(Math.pow(tran.get(0), 2) + Math.pow(tran.get(2), 2));
    }

    public static double getMoveAngle(VuforiaTrackableDefaultListener listener, VectorF vector){
        VectorF angles = anglesFromTarget(listener);

        VectorF tran = navOffWall(listener.getPose().getTranslation(), Math.toDegrees(angles.get(0)), vector);

        return Math.atan2(tran.get(0), tran.get(2)) - Math.PI / 2;
    }

    public static VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall){
        return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    private static VectorF anglesFromTarget(VuforiaTrackableDefaultListener image){
        float [] data = image.getRawPose().getData();
        float [] [] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};

        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
        return new VectorF((float)thetaX, (float)thetaY, (float)thetaZ);
    }
}
