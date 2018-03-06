package org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.Auto.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Vuforia.PineappleRelicRecoveryVuforia;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources.RelicRecoveryConfigV2;

/**
 * Created by ftcpi on 11/16/2017.
 */
@Autonomous(name = "Jewel")
@Disabled
public class RelicRecoveryJewel extends RelicRecoveryConfigV2 {
    @Override
    public void runOpMode() throws InterruptedException {
        config(this);
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = "AdB8VB7/////AAAAGcfBp9I80URFkfBQFUyM+ptmQXBAMGx0svJKz7QE2nm20mBc/zI5sZNHfuP/ziIm+sYnO7fvPqUbFs8QWjRyXVEDmW4mMj+S+l+yaYRkpGZ/pmHyXiDb4aemHx0m70BulMNIce4+NVaCW5S/5BWNNev/AU0P+uWnHYuKNWzD2dPaRuprC4R6b/DgD1zeio1xlssYb9in9mfzn76gChOrE5B0ql6Q9FiHC5cTdacq2lKjm5nlkTiXz1e2jhVK3SddGoqM4FQ3mFks7/A88hFzlPfIIk45K2Lh7GvcVjuIiqNj5mTLaZJVqlsKdTQnKS4trJcc1YV9sjdbmh1agtn1UePy91fDj9uWSBdXvpIowv4B";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;


        VuforiaLocalizer locale = ClassFactory.createVuforiaLocalizer(params);
        locale.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        VuforiaTrackables relicTrackables;
        relicTrackables = locale.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackableDefaultListener track = (VuforiaTrackableDefaultListener) relicTrackables.get(0).getListener();
        relicTrackables.activate();
        while (track.getPose()==null){
            Thread.sleep(1000);
        }
        PineappleEnum.JewelState lastState = PineappleRelicRecoveryVuforia.getJewelConfig(PineappleRelicRecoveryVuforia.getImageFromFrame(locale.getFrameQueue().take(), PIXEL_FORMAT.RGB565),track, locale.getCameraCalibration(), telemetry);
        PineappleEnum.JewelState state = PineappleRelicRecoveryVuforia.getJewelConfig(PineappleRelicRecoveryVuforia.getImageFromFrame(locale.getFrameQueue().take(), PIXEL_FORMAT.RGB565),track, locale.getCameraCalibration(), telemetry);

        boolean hasJewelConfig = false;
        if (lastState == state) {
            hasJewelConfig = true;
        }
        while (opModeIsActive()&& !hasJewelConfig){
            lastState = state;
            state = PineappleRelicRecoveryVuforia.getJewelConfig(PineappleRelicRecoveryVuforia.getImageFromFrame(locale.getFrameQueue().take(), PIXEL_FORMAT.RGB565),track, locale.getCameraCalibration(), telemetry);
            if (lastState == state) {
                hasJewelConfig = true;
            }

            Thread.sleep(500);
        }
        if (state == PineappleEnum.JewelState.NON_NON) {
            telemetry.addData("Config ", "NON");
        } else if (state == PineappleEnum.JewelState.BLUE_RED) {
            telemetry.addData("Config ", "BLUE RED");
        } else if (state == PineappleEnum.JewelState.RED_BLUE) {
            telemetry.addData("Config ", "RED BLUE");
        }
        telemetry.update();

        waitForStart();
//        hitJewels(state);

    }
}
