package org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.Auto;//package org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.Auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.vuforia.PIXEL_FORMAT;
//import com.vuforia.Vuforia;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources.RelicRecoveryConstants;
//import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources.RelicRecoveryEnums;
//import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
//import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Vuforia.PineappleRelicRecoveryVuforia;
//
///**
// * Created by young on 9/14/2017.
// */
//
//@Autonomous(name = "RelicRecoveryAuto", group = "Linear Opmode")
//@Disabled
//public class RelicRecoveryAutonomous extends RelicRecoveryAbstractAutonomous {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry.addLine("Init-Started");
//        telemetry.addLine("Init-Config");
//        telemetry.update();
//        config(this);
//
//
//        //VUFORIA set up
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        parameters.vuforiaLicenseKey = "AdB8VB7/////AAAAGcfBp9I80URFkfBQFUyM+ptmQXBAMGx0svJKz7QE2nm20mBc/zI5sZNHfuP/ziIm+sYnO7fvPqUbFs8QWjRyXVEDmW4mMj+S+l+yaYRkpGZ/pmHyXiDb4aemHx0m70BulMNIce4+NVaCW5S/5BWNNev/AU0P+uWnHYuKNWzD2dPaRuprC4R6b/DgD1zeio1xlssYb9in9mfzn76gChOrE5B0ql6Q9FiHC5cTdacq2lKjm5nlkTiXz1e2jhVK3SddGoqM4FQ3mFks7/A88hFzlPfIIk45K2Lh7GvcVjuIiqNj5mTLaZJVqlsKdTQnKS4trJcc1YV9sjdbmh1agtn1UePy91fDj9uWSBdXvpIowv4B";
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//        vuforia.setFrameQueueCapacity(1);
//        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
//        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
//        VuforiaTrackable relicTemplate = relicTrackables.get(0);
//        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
//        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) relicTemplate.getListener();
//        VuforiaTrackableDefaultListener track = (VuforiaTrackableDefaultListener) relicTrackables.get(0).getListener();
//
//        relicTrackables.activate();
//        telemetry.addLine("Init-Vuforia Initialized");
//
//
//        loadSwitchBoard();
//
//        telemetry.addLine("Init-Complete");
//        telemetry.update();
//        int val = 1;
//        while (listener.getPose() == null && !opModeIsActive() && !isStopRequested()) {
//            if (val == 5) {
//                val = 1;
//            }
//            String dots = "";
//            for (int i = 0; i < val; i++) {
//                dots += ".";
//            }
//            telemetry.addLine("Finding image" + dots);
//            telemetry.update();
//            Thread.sleep(300);
//            val++;
//        }
//        RelicRecoveryVuMark keyColumn = RelicRecoveryVuMark.UNKNOWN;
//        while (!opModeIsActive() && !isStopRequested() && keyColumn == RelicRecoveryVuMark.UNKNOWN) {
//            keyColumn = RelicRecoveryVuMark.from(relicTemplate);
//            if (val == 5) {
//                val = 1;
//            }
//            String dots = "";
//            for (int i = 0; i < val; i++) {
//                dots += ".";
//            }
//            telemetry.addLine("Finding Key Column" + dots);
//            telemetry.update();
//            Thread.sleep(300);
//            val++;
//        }
//
//        phoneTurnLeft.setPosition(RelicRecoveryConstants.PHONELEFTJEWELS);
//        phoneTurnRight.setPosition(RelicRecoveryConstants.PHONERIGHTJEWELS);
//        PineappleEnum.JewelState state = PineappleRelicRecoveryVuforia.getJewelConfig(PineappleRelicRecoveryVuforia.getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), track, vuforia.getCameraCalibration(), telemetry);
//        int i = 1;
//        while (!opModeIsActive() && !isStopRequested()) {
//            state = PineappleRelicRecoveryVuforia.getJewelConfig(PineappleRelicRecoveryVuforia.getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), track, vuforia.getCameraCalibration(), telemetry);
//
//
//            switch (state) {
//                case NON_NON:
//                    telemetry.addData("Config " + i + ": ", "NON NON");
//                    break;
//                case BLUE_RED:
//                    telemetry.addData("Config " + i + ": ", "BLUE RED");
//                    break;
//                case RED_BLUE:
//                    telemetry.addData("Config " + i + ": ", "RED BLUE");
//                    break;
//            }
//            if (!jewelsEnabled) {
//                telemetry.addLine("JEWELS DISABLED");
//            }
//            telemetry.update();
//            Thread.sleep(500);
//        }
//        //////////////////
//        //WAIT FOR START//
//        //////////////////
//        waitForStart();
//
//        flipTopOut();
//
//        beginningDelay();
//        hitJewels(state);
//        ElapsedTime elapsedTime = new ElapsedTime();
//        switch (position) {
////            case RelicRecoveryEnums.StartingPosition.FRONT:
////                switch (color) {
////
////                    case RelicRecoveryEnums.AutoColor.RED:
////                        //Red Front
////                        elapsedTime.reset();                        //Drive forward
////                        robotHandler.drive.mecanum.setPower(-.3, .3);
////                        while (elapsedTime.milliseconds() < 2000) {
//////                            servoCorrectForPicture(phoneTurnRight, PineappleRelicRecoveryVuforia.getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), track, vuforia.getCameraCalibration(), telemetry, 0.07);
//////                            telemetry.update();
////                        }
////                        robotHandler.drive.stop();
////                        while (!alignWithGyro(90, 0.2) && opModeIsActive()) {
////                            servoCorrectForPicture(phoneTurnRight, PineappleRelicRecoveryVuforia.getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), track, vuforia.getCameraCalibration(), telemetry, 0.04);
////
////                        }
////                        phoneTurnRight.setPosition(RelicRecoveryConstants.PHONERIGHTANGLEDSIDE);
////                        Thread.sleep(500);
////                        //align to cryptobox
////                        if (glyphsEnabled) {
////                            VectorF vector = (keyColumn == RelicRecoveryVuMark.LEFT) ? RelicRecoveryConstants.REDSIDELEFT : (keyColumn == RelicRecoveryVuMark.CENTER) ? RelicRecoveryConstants.REDSIDECENTER : RelicRecoveryConstants.REDSIDERIGHT;
////                            alignToCrypto(90, listener, vector);
////                            conveyRight.setPower(1);
////                            conveyLeft.setPower(-1);
////                            Thread.sleep(4000);
////                            conveyRight.setPower(0);
////                            conveyLeft.setPower(0);
////                        }
//
//                        break;
//                    case RelicRecoveryEnums.AutoColor.BLUE:
//                        //Blue Front
//
//
//                        elapsedTime.reset();                        //Drive forward
//                        robotHandler.drive.mecanum.setPower(-.3, .3);
//                        while (elapsedTime.milliseconds() < 2000) {
////                            servoCorrectForPicture(phoneTurnLeft, PineappleRelicRecoveryVuforia.getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), track, vuforia.getCameraCalibration(), telemetry, 0.02);
////                            telemetry.update();
//                        }
//                        robotHandler.drive.stop();
////                        while (!alignWithGyro(270, 0.1) && opModeIsActive()) {
////
////                        }
////                        phoneTurnLeft.setPosition(RelicRecoveryConstants.PHONELEFTANGLEDSIDE);
////                        Thread.sleep(500);
////                        //align to cryptobox
////                        if (glyphsEnabled) {
////                            VectorF vector = (keyColumn == RelicRecoveryVuMark.LEFT) ? RelicRecoveryConstants.BLUESIDELEFT : (keyColumn == RelicRecoveryVuMark.CENTER) ? RelicRecoveryConstants.BLUESIDECENTER : RelicRecoveryConstants.BLUESIDERIGHT;
////                            alignToCrypto(-90, listener, vector);
////                            conveyRight.setPower(1);
////                            conveyLeft.setPower(-1);
////                            Thread.sleep(4000);
////                            conveyRight.setPower(0);
////                            conveyLeft.setPower(0);
////                        }
//                        break;
//
//                    //TODO SPIN out box
//                }
//                break;
//            case RelicRecoveryEnums.StartingPosition.BACK:
//                switch (color) {
//
//                    case RelicRecoveryEnums.AutoColor.RED:
//                        //Red Front
//                        elapsedTime.reset();                        //Drive forward
//                        robotHandler.drive.mecanum.setMecanum(135,0.2,0,1);
//                        while (elapsedTime.milliseconds() < 3000) {
////                            servoCorrectForPicture(phoneTurnRight, PineappleRelicRecoveryVuforia.getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), track, vuforia.getCameraCalibration(), telemetry, 0.02);
////                            telemetry.update();
//                        }
//                        robotHandler.drive.stop();
////                        phoneTurnRight.setPosition(RelicRecoveryConstants.PHONERIGHTANGLEDBACK);
////                        Thread.sleep(500);
////                        //align to cryptobox
////                        if (glyphsEnabled) {
////                            VectorF vector = (keyColumn == RelicRecoveryVuMark.LEFT) ? RelicRecoveryConstants.REDBACKLEFT : (keyColumn == RelicRecoveryVuMark.CENTER) ? RelicRecoveryConstants.REDBACKCENTER : RelicRecoveryConstants.REDBACKRIGHT;
////                            alignToCrypto(90, listener, vector);
////                            conveyRight.setPower(1);
////                            conveyLeft.setPower(-1);
////                            Thread.sleep(4000);
////                            conveyRight.setPower(0);
////                            conveyLeft.setPower(0);
////                        }
//
//                        break;
//                    case RelicRecoveryEnums.AutoColor.BLUE:
//                        //Blue Front
//
//
//                        elapsedTime.reset();                        //Drive forward
//                        robotHandler.drive.mecanum.setMecanum(45,0.2,0,1);
//                        while (elapsedTime.milliseconds() < 3000) {
////                            servoCorrectForPicture(phoneTurnLeft, PineappleRelicRecoveryVuforia.getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), track, vuforia.getCameraCalibration(), telemetry, 0.02);
////                            telemetry.update();
//                        }
//                        robotHandler.drive.stop();
////                        while (!alignWithGyro(270, 0.1) && opModeIsActive()) {
////                            servoCorrectForPicture(phoneTurnLeft, PineappleRelicRecoveryVuforia.getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), track, vuforia.getCameraCalibration(), telemetry, 0.02);
////
////                        }
////                        phoneTurnLeft.setPosition(RelicRecoveryConstants.PHONELEFTANGLEDBACK);
////                        Thread.sleep(500);
////                        //align to cryptobox
////                        if (glyphsEnabled) {
////                            VectorF vector = (keyColumn == RelicRecoveryVuMark.LEFT) ? RelicRecoveryConstants.BLUEBACKLEFT : (keyColumn == RelicRecoveryVuMark.CENTER) ? RelicRecoveryConstants.BLUEBACKCENTER : RelicRecoveryConstants.BLUEBACKRIGHT;
////                            alignToCrypto(-90, listener, vector);
////                            conveyRight.setPower(1);
////                            conveyLeft.setPower(-1);
////                            Thread.sleep(4000);
////                            conveyRight.setPower(0);
////                            conveyLeft.setPower(0);
////                        }
//                        break;
//
//                    //TODO SPIN out box
//                }
//                break;
//        }
//
//
//    }
//
//
//}
