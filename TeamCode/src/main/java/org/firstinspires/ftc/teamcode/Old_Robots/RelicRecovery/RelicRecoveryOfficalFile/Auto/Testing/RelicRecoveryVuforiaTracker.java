package org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.Auto.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources.RelicRecoveryConstants;
import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources.RelicRecoveryConfigV2;

/**
 * Created by Brandon on 11/28/2017.
 */
@TeleOp(name = "VuforiaGetValue",group = "Linear Opmode")
@Disabled
public class RelicRecoveryVuforiaTracker extends RelicRecoveryConfigV2 {
    @Override
    public void runOpMode() throws InterruptedException {
        config(this);

        telemetry.addLine("Init");
        telemetry.update();
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


        waitForStart();
        phoneTurnLeft.setPosition(RelicRecoveryConstants.PHONELEFTANGLEDSIDE);
        phoneTurnRight.setPosition(RelicRecoveryConstants.PHONERIGHTANGLEDSIDE);
        while (opModeIsActive()) {
            if (listener.getPose() != null) {
                VectorF trans = listener.getPose().getTranslation();
                double[] robot = {trans.get(1), trans.get(2)};

                String vuMark = (RelicRecoveryVuMark.from(relicTemplate) == RelicRecoveryVuMark.LEFT) ? "Left" : (RelicRecoveryVuMark.from(relicTemplate) == RelicRecoveryVuMark.CENTER) ? "Center" : "Right";
                telemetry.addData("VuMark", vuMark);
                telemetry.addData("X-1", robot[0]);
                telemetry.addData("Y-2", robot[1]);
            } else {
                telemetry.addData("VuMark", "No Visible");
            }
            telemetry.update();

        }
    }
}