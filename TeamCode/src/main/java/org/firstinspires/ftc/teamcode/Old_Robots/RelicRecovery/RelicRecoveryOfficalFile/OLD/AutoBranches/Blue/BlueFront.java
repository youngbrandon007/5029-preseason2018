package org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.OLD.AutoBranches.Blue;

import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.Auto.RelicRecoveryAbstractAutonomous;

/**
 * Created by Brandon on 11/14/2017.
 */

public class BlueFront extends RelicRecoveryAbstractAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("OpMode", "Blue");
        telemetry.update();
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        parameters.vuforiaLicenseKey = "AdB8VB7/////AAAAGcfBp9I80URFkfBQFUyM+ptmQXBAMGx0svJKz7QE2nm20mBc/zI5sZNHfuP/ziIm+sYnO7fvPqUbFs8QWjRyXVEDmW4mMj+S+l+yaYRkpGZ/pmHyXiDb4aemHx0m70BulMNIce4+NVaCW5S/5BWNNev/AU0P+uWnHYuKNWzD2dPaRuprC4R6b/DgD1zeio1xlssYb9in9mfzn76gChOrE5B0ql6Q9FiHC5cTdacq2lKjm5nlkTiXz1e2jhVK3SddGoqM4FQ3mFks7/A88hFzlPfIIk45K2Lh7GvcVjuIiqNj5mTLaZJVqlsKdTQnKS4trJcc1YV9sjdbmh1agtn1UePy91fDj9uWSBdXvpIowv4B";
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//
//
//
//        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
//        VuforiaTrackable relicTemplate = relicTrackables.get(0);
//        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
//
//        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) relicTemplate.getListener();
//
//        relicTrackables.activate();

        waitForStart();

//        alignToCrypto(listener,new VectorF(-500, 0 ,0));


        while (opModeIsActive()&&navx_device.isConnected()) {

            double gyroAngle = this.navx_device.getYaw();
            gyroAngle+=180;
            telemetry.addData("Gyro Angle", gyroAngle);
            telemetry.update();
            double rotation;
            if (gyroAngle > 1 && gyroAngle < 180) {
                //Put Gyro here
                rotation = .1;
                robotHandler.drive.tank.setPower(rotation, rotation);
            } else if (gyroAngle < 359 && gyroAngle > 179) {
                //and here
                rotation = -.1;
                robotHandler.drive.tank.setPower(rotation, rotation);

            }
        }
    }
}
