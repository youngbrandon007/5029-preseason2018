package org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFileAfterWV.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleRobotConstants;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Vuforia.PineappleRelicRecoveryVuforia;
import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources.RelicRecoveryConstants;
import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources.RelicRecoveryEnums;
import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFileAfterWV.RelicRecoveryResources.RelicRecoveryConfigV2Cleve;

import static org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum.JewelState.BLUE_RED;
import static org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum.JewelState.RED_BLUE;

/**
 * Created by ftcpi on 1/3/2018.
 */
@Autonomous(name = "Auto Clev")
public class RelicRecoveryAutonomousMainVCLEV extends RelicRecoveryConfigV2Cleve {
    enum Init {
        CALIBGYRO, FINDIMAGE, FINDKEY, GETJEWELCONFIG
    }

    enum Auto {
        JEWELDOWN, JEWELTURN, JEWELUP, DRIVEOFFPLAT, DRIVEOFFPLAT2, PDRIVEFORWARD, DRIVEFORWARD, BACKAWAYFROMCRYPTO, TURNTOCRYPTO, STRAFEAWAYFROMCOLUNM, PDRIVEFORWARDTOCRYPTO, DRIVEFORWARDTOCRYPTO, ALIGNTOCRYPTO, DRIVEAWAYFROMCRYPT, PLACEGLYPH, PLACEBACK, PLACEFORWARD, BACKAGAIN;
    }

    Auto auto = Auto.JEWELDOWN;
    Init init = Init.CALIBGYRO;

    @Override
    public void runOpMode() throws InterruptedException {
        config(this);
        loadSwitchBoard();
        //load vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AdB8VB7/////AAAAGcfBp9I80URFkfBQFUyM+ptmQXBAMGx0svJKz7QE2nm20mBc/zI5sZNHfuP/ziIm+sYnO7fvPqUbFs8QWjRyXVEDmW4mMj+S+l+yaYRkpGZ/pmHyXiDb4aemHx0m70BulMNIce4+NVaCW5S/5BWNNev/AU0P+uWnHYuKNWzD2dPaRuprC4R6b/DgD1zeio1xlssYb9in9mfzn76gChOrE5B0ql6Q9FiHC5cTdacq2lKjm5nlkTiXz1e2jhVK3SddGoqM4FQ3mFks7/A88hFzlPfIIk45K2Lh7GvcVjuIiqNj5mTLaZJVqlsKdTQnKS4trJcc1YV9sjdbmh1agtn1UePy91fDj9uWSBdXvpIowv4B";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        vuforia.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) relicTemplate.getListener();
        //This is not needed as it is a repeat from the line above


        relicTrackables.activate();

        RelicRecoveryVuMark keyColumn = RelicRecoveryVuMark.UNKNOWN;

        int val = 1;
        int x = 0;
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("Init");
            switch (init) {
                case CALIBGYRO:

                    calibration_complete = !navx_device.isCalibrating();
                    if (!calibration_complete) {
                        telemetry.addData("navX-Micro", "Startup Calibration in Progress");
                    } else {
                        navx_device.zeroYaw();
                        init = Init.FINDIMAGE;
                        telemetry.addData("navX-Micro", "Calibration Finished");
                    }
                    break;
                case FINDIMAGE:
                    val++;
                    if (val == 5) {
                        val = 1;
                    }
                    String dots = "";
                    for (int i = 0; i < val; i++) {
                        dots += ".";
                    }
                    if (listener.getPose() == null) {
                        telemetry.addLine("Finding image" + dots);
                    } else {
                        init = Init.FINDKEY;
                    }
                    Thread.sleep(100);
                    break;
                case FINDKEY:
                    if (listener.getPose() != null) {
                        keyColumn = RelicRecoveryVuMark.from(relicTemplate);
                    }
                    init = Init.GETJEWELCONFIG;
                    break;
                case GETJEWELCONFIG:
                    state = PineappleRelicRecoveryVuforia.getJewelConfig(PineappleRelicRecoveryVuforia.getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), listener, vuforia.getCameraCalibration(), telemetry);

                    x++;
                    switch (state) {
                        case NON_NON:
                            telemetry.addData("Config " + x + ": ", "NON NON");
                            break;
                        case BLUE_RED:
                            state = RED_BLUE;
                            telemetry.addData("Config " + x + ": ", "RED BLUE");
                            break;
                        case RED_BLUE:
                            telemetry.addData("Config " + x + ": ", "BLUE RED");
                            state = BLUE_RED;
                            break;
                    }

                    break;
            }

            telemetry.update();
        }
        waitForStart();
        releaseLeft.setPosition(RelicRecoveryConstants.FLIPOUTLEFT);
        releaseRight.setPosition(RelicRecoveryConstants.FLIPOUTRIGHT);
        Thread.sleep(3000);
        releaseLeft.setPosition(RelicRecoveryConstants.FLIPINLEFT);
        releaseRight.setPosition(RelicRecoveryConstants.FLIPINRIGHT);
        auto = Auto.JEWELDOWN;

        wait.reset();

        while (wait.milliseconds() < 5000){

        }

        double startingPos = 0;
        while (opModeIsActive()) {
            telemetry.addData("wait", wait.milliseconds());
            telemetry.addData("STATE", auto);
            telemetry.addData("GYRO_YAW", getHeading());
            telemetry.addData("Column", keyColumn);
            switch (color) {
                case RED:

                    switch (auto) {
                        case JEWELDOWN:
                            if (jewelDown()) {
                                auto = Auto.JEWELTURN;
                                wait.reset();
                            }
                            break;
                        case JEWELTURN:
                            if (jewelTurn(300, 0.15)) {
                                robotHandler.drive.stop();
                                auto = Auto.DRIVEOFFPLAT;
                            }
                            break;
                        case DRIVEOFFPLAT:
                            robotHandler.drive.mecanum.setPower(.2, -.2);
                            telemetry.addData("roll", getRoll());
                            if (getRoll() < -3) {
                                auto = Auto.DRIVEOFFPLAT2;
                            }
                            break;
                        case DRIVEOFFPLAT2:
                            telemetry.addData("roll", getRoll());
                            if (getRoll() > -2) {
                                robotHandler.drive.stop();
                                auto = Auto.PDRIVEFORWARD;
                            }
                            break;

                        case PDRIVEFORWARD:
                            startingPos = getEncoder();

                            robotHandler.drive.mecanum.setPower(.2, -.2);
                            auto = Auto.DRIVEFORWARD;
                            wait.reset();
                            break;
                        case DRIVEFORWARD:
                            double pos = getEncoder();
                            double dis = 250;

                            double cir = 4 * Math.PI;
                            double goSixInch = 7.63 / cir * PineappleRobotConstants.NEV40CPR;
                            goSixInch *= (2.0 / 3.0);
                            if(wait.milliseconds() > 7000){
                                robotHandler.drive.stop();
                                stop();
                            }
                            switch (keyColumn) {
                                case UNKNOWN:
                                    keyColumn = RelicRecoveryVuMark.CENTER;
                                    break;
                                case LEFT:
                                    if (Math.abs(pos - startingPos) > dis + (2 * goSixInch))
                                        auto = Auto.TURNTOCRYPTO;
                                    break;
                                case CENTER:
                                    if (Math.abs(pos - startingPos) > dis + goSixInch)
                                        auto = Auto.TURNTOCRYPTO;
                                    break;
                                case RIGHT:
                                    if (Math.abs(pos - startingPos) > dis)
                                        auto = Auto.TURNTOCRYPTO;
                                    break;
                            }
                            break;
                        case TURNTOCRYPTO:
                            alignLeft.setPosition(RelicRecoveryConstants.ALIGNDOWNLEFT);
                            if (turnTo(270, -.3)) {
                                startingPos = getEncoder();
                                auto = Auto.DRIVEFORWARDTOCRYPTO;
                                wait.reset();
                            }
                            break;
                        case STRAFEAWAYFROMCOLUNM:
                            break;
                        case PDRIVEFORWARDTOCRYPTO:
                            //unneeded not used
                            startingPos = getEncoder();

                            break;
                        case DRIVEFORWARDTOCRYPTO:
                            robotHandler.drive.mecanum.setPower(-.2, .2);
                            if (Math.abs(getEncoder() - startingPos) > 1300 || wait.milliseconds() > 3000) {
                                robotHandler.drive.stop();
                                auto = Auto.BACKAWAYFROMCRYPTO;
                                wait.reset();
                            }
                            break;
                        case BACKAWAYFROMCRYPTO:
                            robotHandler.drive.mecanum.setPower(.2, -.2);
                            if (wait.milliseconds() > 100) {
                                auto = Auto.ALIGNTOCRYPTO;
                            }
                            break;
                        case ALIGNTOCRYPTO:
                            if (alignCrypto()) {
                                robotHandler.drive.stop();
                                startingPos = getEncoder();
                                auto = Auto.DRIVEAWAYFROMCRYPT;
                            }
                            break;
                        case DRIVEAWAYFROMCRYPT:
                            robotHandler.drive.mecanum.setPower(.2, -.2);
                            if (Math.abs(getEncoder() - startingPos) > 150) {
                                robotHandler.drive.stop();
                                alignLeft.setPosition(RelicRecoveryConstants.ALIGNUPLEFT);
                                wait.reset();
                                auto = Auto.PLACEGLYPH;
                            }
                            break;
                        case PLACEGLYPH:
                            conveyRight.setPower(1);
                            conveyLeft.setPower(-1);
                            telemetry.addData("WAITING", wait.milliseconds());
                            if (wait.milliseconds() > 3000) {
                                auto = Auto.PLACEFORWARD;
                                startingPos = getEncoder();
                                wait.reset();
                            }
                            break;
                        case PLACEFORWARD:
                            robotHandler.drive.mecanum.setPower(.2, -.2);
                            if (Math.abs(getEncoder() - startingPos) > 200 || wait.milliseconds() > 4000) {
                                auto = Auto.PLACEBACK;
                                startingPos = getEncoder();
                            }
                            break;
                        case PLACEBACK:
                            robotHandler.drive.mecanum.setPower(-.2, .2);
                            if (Math.abs(getEncoder() - startingPos) > 200) {
                                robotHandler.drive.stop();
                                auto = Auto.BACKAGAIN;
                                startingPos = getEncoder();
                            }
                            break;


                        case BACKAGAIN:
                            robotHandler.drive.mecanum.setPower(.2, -.2);
                            if (Math.abs(getEncoder() - startingPos) > 200) {
                                robotHandler.drive.stop();
                                conveyRight.setPower(0);
                                conveyLeft.setPower(0);
                                stop();
                            }
                            break;
                    }
                    break;
                case BLUE:

                    switch (auto) {
                        case JEWELDOWN:
                            if (jewelDown()) {
                                auto = Auto.JEWELTURN;
                                wait.reset();
                            }
                            break;
                        case JEWELTURN:
                            if (jewelTurn(300, 0.15)) {
                                robotHandler.drive.stop();
                                auto = Auto.DRIVEOFFPLAT;
                            }
                            break;
                        case DRIVEOFFPLAT:
                            robotHandler.drive.mecanum.setPower(-.2, .2);
                            if (getRoll() > 4) {
                                auto = Auto.DRIVEOFFPLAT2;
                            }
                            break;
                        case DRIVEOFFPLAT2:
                            if (getRoll() < 2) {
                                robotHandler.drive.stop();
                                auto = Auto.PDRIVEFORWARD;
                            }
                            break;
                        case PDRIVEFORWARD:
                            startingPos = getEncoder();

                            robotHandler.drive.mecanum.setPower(-.2, .2);
                            auto = Auto.DRIVEFORWARD;
                            wait.reset();
                            break;
                        case DRIVEFORWARD:
                            double pos = getEncoder();
                            double dis = 50;

                            double cir = 4 * Math.PI;
                            double goSixInch = 7.63 / cir * PineappleRobotConstants.NEV40CPR;
                            goSixInch *= (2.0 / 3.0);
                            if(wait.milliseconds() > 7000){
                                robotHandler.drive.stop();
                                stop();
                            }
                            switch (keyColumn) {
                                case UNKNOWN:
                                    keyColumn = RelicRecoveryVuMark.CENTER;
                                    wait.reset();

                                    break;
                                case RIGHT:
                                    if (Math.abs(pos - startingPos) > dis + (2 * goSixInch))
                                        auto = Auto.TURNTOCRYPTO;
                                    wait.reset();

                                    break;
                                case CENTER:
                                    if (Math.abs(pos - startingPos) > dis + goSixInch)
                                        auto = Auto.TURNTOCRYPTO;
                                    wait.reset();

                                    break;
                                case LEFT:
                                    if (Math.abs(pos - startingPos) > dis)
                                        auto = Auto.TURNTOCRYPTO;
                                    wait.reset();
                                    break;
                            }
                            break;

                        case TURNTOCRYPTO:
                            alignLeft.setPosition(RelicRecoveryConstants.ALIGNDOWNLEFT);
                            if (turnTo(270, -.3)) {
                                wait.reset();
                                auto = Auto.STRAFEAWAYFROMCOLUNM;
                            }
                            break;
                        case STRAFEAWAYFROMCOLUNM:
                            robotHandler.drive.mecanum.setMecanum(Math.PI, 0.2, 0, 1);
                            if (wait.milliseconds() > 200) {
                                auto = Auto.PDRIVEFORWARDTOCRYPTO;
                            }
                            break;
                        case PDRIVEFORWARDTOCRYPTO:
                            startingPos = getEncoder();
                            auto = Auto.DRIVEFORWARDTOCRYPTO;
                            break;
                        case DRIVEFORWARDTOCRYPTO:
                            robotHandler.drive.mecanum.setPower(-.2, .2);
                            if (Math.abs(getEncoder() - startingPos) > 800 || wait.milliseconds() > 3000) {
                                robotHandler.drive.stop();
                                auto = Auto.BACKAWAYFROMCRYPTO;
                            }
                            break;
                        case BACKAWAYFROMCRYPTO:
                            robotHandler.drive.mecanum.setPower(.2, -.2);
                            if (wait.milliseconds() > 200) {
                                robotHandler.drive.stop();
                                wait.reset();
                                auto = Auto.ALIGNTOCRYPTO;
                            }
                            break;
                        case ALIGNTOCRYPTO:
                            if (alignCrypto()) {
                                robotHandler.drive.stop();
                                startingPos = getEncoder();
                                auto = Auto.DRIVEAWAYFROMCRYPT;
                            }
                            break;

                        case DRIVEAWAYFROMCRYPT:
                            robotHandler.drive.mecanum.setPower(.2, -.2);
                            if (Math.abs(getEncoder() - startingPos) > 150) {
                                robotHandler.drive.stop();
                                alignLeft.setPosition(RelicRecoveryConstants.ALIGNUPLEFT);
                                wait.reset();
                                auto = Auto.PLACEGLYPH;
                            }
                            break;
                        case PLACEGLYPH:
                            conveyRight.setPower(1);
                            conveyLeft.setPower(-1);
                            telemetry.addData("WAITING", wait.milliseconds());
                            if (wait.milliseconds() > 3000) {
                                auto = Auto.PLACEFORWARD;
                                startingPos = getEncoder();
                            }
                            break;
                        case PLACEFORWARD:
                            robotHandler.drive.mecanum.setPower(.2, -.2);
                            if (Math.abs(getEncoder() - startingPos) > 200) {
                                auto = Auto.PLACEBACK;
                                startingPos = getEncoder();
                                wait.reset();
                            }
                            break;
                        case PLACEBACK:
                            robotHandler.drive.mecanum.setPower(-.2, .2);
                            if (Math.abs(getEncoder() - startingPos) > 200 || wait.milliseconds() > 4000) {
                                robotHandler.drive.stop();
                                auto = Auto.BACKAGAIN;
                                startingPos = getEncoder();
                            }
                            break;


                        case BACKAGAIN:
                            robotHandler.drive.mecanum.setPower(.2, -.2);
                            if (Math.abs(getEncoder() - startingPos) > 200) {
                                robotHandler.drive.stop();
                                conveyRight.setPower(0);
                                conveyLeft.setPower(0);
                                stop();
                            }
                            break;
                    }
                    break;
            }
            telemetry.update();
        }
    }

    public boolean turnTo(double angle, double speed) {
        double heading = getHeading();
        double target = heading - angle;
        target += (target < 0) ? 360 : 0;


        telemetry.addData("Head", heading);
        telemetry.addData("Traveling", target);

        if (target < 182 && target > 178) {
            robotHandler.drive.stop();
            return true;
        } else if (target < 220 && target > 140) {
            robotHandler.drive.mecanum.setPower(speed / 2, speed / 2);
        } else {
            robotHandler.drive.mecanum.setPower(speed, speed);
        }
        return false;
    }

    public int getEncoder() {
        return (int) driveFrontLeft.getEncoderPosition();
    }


    public double getHeading() {
        return navx_device.getYaw() + 180;
    }

    public double getRoll() {
        return navx_device.getRoll();
    }

    public boolean alignCrypto() {
        robotHandler.drive.mecanum.setMecanum(Math.toRadians(180), 0.6, 0, 1);
        return cryptoTouchSensor.getValue(PineappleEnum.PineappleSensorEnum.TOUCH) == 1;
    }

    public boolean jewelDown() {
        jewel.setPosition(jewel.servoObject.getPosition() + 0.01);
        return jewel.servoObject.getPosition() > RelicRecoveryConstants.JEWELDOWN;
    }

    public boolean jewelUp() {
        jewel.setPosition(jewel.servoObject.getPosition() - 0.01);
        return jewel.servoObject.getPosition() <= RelicRecoveryConstants.JEWELUP;
    }

    public boolean jewelTurn(double turnAmount, double power) {
        double rotation = 0;


        if (color == RelicRecoveryEnums.AutoColor.RED) {
            switch (state) {
                case BLUE_RED:
                    rotation = power;
                    break;
                case RED_BLUE:
                    rotation = -power;
                    break;
                case NON_NON:
            }
        } else {
            switch (state) {
                case BLUE_RED:
                    rotation = -power;
                    break;
                case RED_BLUE:
                    rotation = power;
                    break;
                case NON_NON:
            }
        }
        if (wait.milliseconds() > turnAmount) {
            rotation = 0;
            if (jewelUp() && wait.milliseconds() > (turnAmount) *2) {
                rotation *= -1;
                if (wait.milliseconds() > (turnAmount) * 3) {

                    return true;
                }
            }

        }

        robotHandler.drive.mecanum.setPower(-rotation, -rotation);
        turnCount++;
        return false;

    }
}
