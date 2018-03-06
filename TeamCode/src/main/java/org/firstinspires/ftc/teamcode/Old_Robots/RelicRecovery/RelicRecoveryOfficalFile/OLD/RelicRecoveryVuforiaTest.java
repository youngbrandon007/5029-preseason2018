package org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.OLD;

/**
 * Created by Brandon on 10/15/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources.RelicRecoveryConfig;

@TeleOp(name="KrishnaVuforiaTest", group ="Concept")
@Disabled
public class RelicRecoveryVuforiaTest extends RelicRecoveryConfig {

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() throws InterruptedException {

        config(this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


        parameters.vuforiaLicenseKey = "AdB8VB7/////AAAAGcfBp9I80URFkfBQFUyM+ptmQXBAMGx0svJKz7QE2nm20mBc/zI5sZNHfuP/ziIm+sYnO7fvPqUbFs8QWjRyXVEDmW4mMj+S+l+yaYRkpGZ/pmHyXiDb4aemHx0m70BulMNIce4+NVaCW5S/5BWNNev/AU0P+uWnHYuKNWzD2dPaRuprC4R6b/DgD1zeio1xlssYb9in9mfzn76gChOrE5B0ql6Q9FiHC5cTdacq2lKjm5nlkTiXz1e2jhVK3SddGoqM4FQ3mFks7/A88hFzlPfIIk45K2Lh7GvcVjuIiqNj5mTLaZJVqlsKdTQnKS4trJcc1YV9sjdbmh1agtn1UePy91fDj9uWSBdXvpIowv4B";


        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);



        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) relicTemplate.getListener();
        waitForStart();



        relicTrackables.activate();

        while (opModeIsActive()) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getRawPose();
                //telemetry.addData("Pose", format(pose));


                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

// Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;




                    if(listener.getPose() != null) {
                        VectorF angles = anglesFromTarget(listener);
                        telemetry.addData("pose 0", Math.toDegrees(angles.get(0)) + 180);

                        VectorF tran = navOffWall(listener.getPose().getTranslation(), Math.toDegrees(angles.get(0)), new VectorF(-500, 0, 0));
                        double tranAngle = Math.atan2(tran.get(0), tran.get(2)) - Math.PI/2;
                        double distance = Math.sqrt(Math.pow(tran.get(0),2)+Math.pow(tran.get(2),2));
                        double rotation = 0;
                        double angle = Math.toDegrees(angles.get(0)) + 180;
                        if (angle > 2 && angle < 180) {
                            rotation = .1;
                            robotHandler.drive.tank.setPower(rotation, rotation);
                        } else if (angle < 358 && angle > 179) {
                            rotation = -.1;
                            robotHandler.drive.tank.setPower(rotation, rotation);
                        }else{
                            if(distance > 20) {
                                robotHandler.drive.mecanum.setMecanum(tranAngle, .5, 0 , 1);
                            }
                        }

                    }


//                    }else if(gamepad1.b && Math.abs(angle) > 5 ) {
//                        if (angle < 0) {
//                            robotHandler.drive.mecanum.setMecanum(0, .5, .5, 1);
//                        } else {
//                            robotHandler.drive.mecanum.setMecanum(0, .5, -.5, 1);
//                        }
//                    }else if(gamepad1.x){
//                        robotHandler.drive.stop();
//
//                        angles = anglesFromTarget(listener);
//                        tran = navOffWall(listener.getPose().getTranslation(), Math.toDegrees(angles.get(0)),new VectorF(-500, 0, 0));
//
//                        distance = Math.sqrt(Math.pow(tran.get(0),2) + Math.pow(tran.get(2),2));
//                        angle = Math.atan2(tran.get(0), tran.get(2));
//
//                        String dis = Double.toString(distance * 10) + "CM";
//
//                        robotHandler.drive.mecanum.encoderMecanum(angle, .5, dis, 4, gyroSensor);
//
//                        while (opModeIsActive() && (listener.getPose() == null || Math.abs(listener.getPose().getTranslation().get(0)) > 10)){
//                            if(listener.getPose() != null){
//                                if(listener.getPose().getTranslation().get(0) > 0){
//                                    robotHandler.drive.mecanum.setPower(-.5, .5);
//                                }else{
//                                    robotHandler.drive.mecanum.setPower(.5, -.5);
//                                }
//                            }else{
//                                robotHandler.drive.mecanum.setPower(-.5, .5);
//                            }
//                        }
//
//                        robotHandler.drive.stop();

                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
                robotHandler.drive.stop();
            }

            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall){
        return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image){
        float [] data = image.getRawPose().getData();
        float [] [] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};

        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
        return new VectorF((float)thetaX, (float)thetaY, (float)thetaZ);
    }

    public void allign(VuforiaTrackableDefaultListener listener) throws InterruptedException {

        boolean go = true;
        double rotation = 0;
        while(go && opModeIsActive()){
            if(listener.getPose() != null){

                double robotAngle = getRobotAngle(listener);
                double distance = getDistance(listener);
                double moveAngle = getMoveAngle(listener);

                //Robot Rotation First
                if (robotAngle > 1 && robotAngle < 180) {
                    rotation = .1;
                    robotHandler.drive.tank.setPower(rotation, rotation);
                } else if (robotAngle < 359 && robotAngle > 179) {
                    rotation = -.1;
                    robotHandler.drive.tank.setPower(rotation, rotation);
                }else{
                    //Robot Movement
                    if(distance > 50) {
                        robotHandler.drive.mecanum.setMecanum(moveAngle, .5, 0, 1);
                    }else if(distance > 5){
                        robotHandler.drive.mecanum.setMecanum(moveAngle, .3, 0, 1);
                        Thread.sleep(100);
                        robotHandler.drive.stop();
                    }else{
                        go = false;
                    }
                }
            }else{
                //Null position of picture
                robotHandler.drive.tank.setPower(.1,-.1);
            }
        }
        robotHandler.drive.stop();
    }

    double getRobotAngle(VuforiaTrackableDefaultListener listener){
        VectorF angles = anglesFromTarget(listener);

        double angle = Math.toDegrees(angles.get(0)) + 180;
        return angle;
    }

    double getDistance(VuforiaTrackableDefaultListener listener){
        VectorF angles = anglesFromTarget(listener);

        VectorF tran = navOffWall(listener.getPose().getTranslation(), Math.toDegrees(angles.get(0)), new VectorF(-500, 0, 0));

        double distance = Math.sqrt(Math.pow(tran.get(0), 2) + Math.pow(tran.get(2), 2));

        return distance;
    }

    double getMoveAngle(VuforiaTrackableDefaultListener listener){
        VectorF angles = anglesFromTarget(listener);

        VectorF tran = navOffWall(listener.getPose().getTranslation(), Math.toDegrees(angles.get(0)), new VectorF(-500, 0, 0));

        double tranAngle = Math.atan2(tran.get(0), tran.get(2)) - Math.PI / 2;
        return tranAngle;
    }

}