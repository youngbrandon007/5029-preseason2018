package org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.OLD;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources.RelicRecoveryConfig;

/**
 * Created by Brandon on 10/31/2017.
 */
@Autonomous(name = "_11/1Test", group = "Linear Opmode")
@Disabled
public class AutonumousTest10_31_2017 extends RelicRecoveryConfig{

    private static final VectorF vector = new VectorF(-700, 0, -700);

    @Override
    public void runOpMode() throws InterruptedException {
        config(this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AdB8VB7/////AAAAGcfBp9I80URFkfBQFUyM+ptmQXBAMGx0svJKz7QE2nm20mBc/zI5sZNHfuP/ziIm+sYnO7fvPqUbFs8QWjRyXVEDmW4mMj+S+l+yaYRkpGZ/pmHyXiDb4aemHx0m70BulMNIce4+NVaCW5S/5BWNNev/AU0P+uWnHYuKNWzD2dPaRuprC4R6b/DgD1zeio1xlssYb9in9mfzn76gChOrE5B0ql6Q9FiHC5cTdacq2lKjm5nlkTiXz1e2jhVK3SddGoqM4FQ3mFks7/A88hFzlPfIIk45K2Lh7GvcVjuIiqNj5mTLaZJVqlsKdTQnKS4trJcc1YV9sjdbmh1agtn1UePy91fDj9uWSBdXvpIowv4B";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);



        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) relicTemplate.getListener();

        relicTrackables.activate();

        while (!isStarted()){
            listener.getPose();
        }

        align(listener,vector);
    }


    //Resources for the Test

    private void align(VuforiaTrackableDefaultListener listener, VectorF vector) throws InterruptedException {

        boolean go = true;

        while(go && opModeIsActive()){
            if(null != listener.getPose()){

                double robotAngle = getRobotAngle(listener) - 45;
                robotAngle += (robotAngle< 0) ? 360: 0;

                double distance = getDistance(listener, vector);
                double moveAngle = getMoveAngle(listener, vector)+90;
                moveAngle -=(moveAngle>360) ? 360:0;
                double rotation;

                telemetry.addData("Angle°", robotAngle);
                telemetry.addData("Distance", distance);
                telemetry.addData("Drive°", moveAngle);
                telemetry.update();
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
                //Null position of picture
                robotHandler.drive.tank.setPower(.1,.1);
            }
        }
        robotHandler.drive.stop();
    }

    private double getRobotAngle(VuforiaTrackableDefaultListener listener){
        VectorF angles = anglesFromTarget(listener);
        double robotAngle = Math.toDegrees(angles.get(0)) + 180;
        return robotAngle;
    }

    private double getDistance(VuforiaTrackableDefaultListener listener, VectorF vector){
        VectorF angles = anglesFromTarget(listener);

        VectorF tran = navOffWall(listener.getPose().getTranslation(), Math.toDegrees(angles.get(0)), vector);

        return Math.sqrt(Math.pow(tran.get(0), 2) + Math.pow(tran.get(2), 2));
    }

    private double getMoveAngle(VuforiaTrackableDefaultListener listener, VectorF vector){
        VectorF angles = anglesFromTarget(listener);

        VectorF tran = navOffWall(listener.getPose().getTranslation(), Math.toDegrees(angles.get(0)), vector);

        return Math.atan2(tran.get(0), tran.get(2)) - Math.PI / 2;
    }

    private VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall){
        return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    private VectorF anglesFromTarget(VuforiaTrackableDefaultListener image){
        float [] data = image.getRawPose().getData();
        float [] [] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};

        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
        return new VectorF((float)thetaX, (float)thetaY, (float)thetaZ);
    }
}
