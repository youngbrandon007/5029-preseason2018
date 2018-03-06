package org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.Auto.RelicRecoveryAbstractAutonomous;

/**
 * Created by young on 1/3/2018.
 */

public class RelicRecoveryRampV3 extends RelicRecoveryAbstractAutonomous {

    public enum Auto{
        SCAN, JEWELS, DRIVEOFFPLATFROM, DRIVETOKEY, TURN, PDRVIEFORWARDTOCRYPTO, DRIVEFORWARDTOCRYPTO, LINEUPTOCRYPTO, PUTGLYPH, PREPARECOLLECT,COLLECT,PREPAREDRIVEBACKTOCRYPTO,DRIVEBACKTOCRYPTO
    }

    ElapsedTime time = new ElapsedTime();
    ElapsedTime totalTime = new ElapsedTime();

    Auto auto = Auto.SCAN;
    RelicRecoveryVuMark keyColumn = RelicRecoveryVuMark.UNKNOWN;
    RelicRecoveryVuMark defaultKeyColumn = RelicRecoveryVuMark.CENTER;

    double facingCrypto = 0.0;

    Position pos = new Position();

    @Override
    public void runOpMode() throws InterruptedException {
        time.reset();
        totalTime.reset();
        telemetry.update();


        //load robot
        config(this);


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
        //VuforiaTrackableDefaultListener track = (VuforiaTrackableDefaultListener) relicTrackables.get(0).getListener();

        relicTrackables.activate();


        //INIT loop
        int val = 1;
        while (!opModeIsActive() && !isStopRequested()) {
            if(listener.getPose() != null) {
                keyColumn = RelicRecoveryVuMark.from(relicTemplate);
            }
            if (val == 5) {
                val = 1;
            }
            String dots = "";
            for (int i = 0; i < val; i++) {
                dots += ".";
            }

            telemetry.addLine("Waiting for Start");
            telemetry.addData("Time(milliseconds)",time.milliseconds());
            if(keyColumn == null){
                telemetry.addLine("Finding Image" + dots);
            }else{
                telemetry.addData("Key Column", keyColumn);
            }
            telemetry.update();

            Thread.sleep(200);
            val++;
        }

        waitForStart();
        time.reset();
        totalTime.reset();

        //testing
        //auto = Auto.DRIVEOFFPLATFROM

        while (opModeIsActive()) {
            telemetry.addData("Time(milliseconds)",totalTime.milliseconds());
            telemetry.addData("Running",auto);
            switch (colorPosition) {
                case REDFRONT:

                    //////////////////
                    //RED FRONT AUTO//
                    //////////////////

                    switch (auto) {
                        case SCAN:
                            //TODO scan jewels
                            break;
                        case JEWELS:
                            //TODO jewels
                            break;
                        case DRIVEOFFPLATFROM:
                            //drive off platform
                            //TODO add encoder drive at the end
                            if (driveOffPlatform(.5))
                                time.reset();
                                auto = Auto.TURN;
                            break;
                        case DRIVETOKEY:
                            //drive forwrd to general position of key
                            //TODO change to encoders
                            robotHandler.drive.mecanum.setPower(-.5,.5);
                            switch (keyColumn){
                                case UNKNOWN:
                                    //Default Key Column is initialezed at the begining of the program
                                    keyColumn = defaultKeyColumn;
                                    break;
                                case LEFT:
                                    if(time.milliseconds() > 500)
                                        auto = Auto.TURN;
                                    break;
                                case CENTER:
                                    if(time.milliseconds() > 1000)
                                        auto = Auto.TURN;
                                    break;
                                case RIGHT:
                                    if(time.milliseconds() > 1500)
                                        auto = Auto.TURN;
                                    break;
                            }
                            break;
                        case TURN:
                            //TODO turn
                            break;
                        case PDRVIEFORWARDTOCRYPTO:
                            time.reset();
                            robotHandler.drive.mecanum.setPower(.5,-.5);
                            auto = Auto.DRIVEFORWARDTOCRYPTO;
                            break;
                        case DRIVEFORWARDTOCRYPTO:
                            //Drive forward a littl to crypto box
                            //TODO might not be needed
                            //TODO change to encoders
                            if(time.milliseconds() > 100){
                                robotHandler.drive.stop();
                                auto = Auto.LINEUPTOCRYPTO;
                            }
                            break;
                        case LINEUPTOCRYPTO:
                            //line up to wall
                            //TODO put servo down-set servo position
                            //TODO make sure this drives left
                            robotHandler.drive.mecanum.setMecanum(Math.toRadians(90), .2,0,1);
                            if(true){//TODO change to button clicked
                                auto = Auto.PUTGLYPH;
                            }
                            break;
                        case PUTGLYPH:
                            //TODO insert glyph
                            break;
                        case PREPARECOLLECT:
                            //only runs once

                            facingCrypto = getHeading();
                            pos.reset();
                            auto = Auto.COLLECT;
                        case COLLECT:
                            //TODO drive to glyph and collect two



                            //calculate position
                            double distance = (driveFrontRight.getEncoderDistance(4) + driveFrontLeft.getEncoderDistance(4) + driveBackRight.getEncoderDistance(4) + driveBackLeft.getEncoderDistance(4))/4;
                            //angle 0 = drinving towards glyphs
                            pos.move(distance ,getHeading() - facingCrypto);
                            telemetry.addData("Position X",pos.x);
                            telemetry.addData("Position Y",pos.y);
                            break;
                        case PREPAREDRIVEBACKTOCRYPTO:
                            //only runs once
                            //TODO stop encoders and
                            auto = Auto.DRIVEBACKTOCRYPTO;
                        case DRIVEBACKTOCRYPTO:
                            //TODO drive back using encoders
                            break;
                        default:
                            break;
                    }
                    break;
                case REDBACK:

                    /////////////////
                    //RED Back AUTO//
                    /////////////////

                    break;
                case BLUEFRONT:

                    ///////////////////
                    //BLUE FRONT AUTO//
                    ///////////////////

                    break;
                case BLUEBACK:


                    //////////////////
                    //BLUE BACK AUTO//
                    //////////////////


                    break;
            }

            telemetry.update();
        }
    }

    private int driveOff = 0;

    public boolean driveOffPlatform(double motorSpeed) {
        robotHandler.drive.mecanum.update(motorSpeed, -motorSpeed);
        double robotTilt = getTilt();
        if (robotTilt > 3) {
            driveOff = 1;
        } else if (robotTilt < 1 && driveOff == 1) {
            time.reset();
            driveOff = 2;
        } else if (driveOff == 2 && time.milliseconds() > 500) {
            return true;
        }
        telemetry.addData("Drive Off", driveOff);
        return false;
    }




    //get Sensors

    public double getTilt() {
        return 0.0;
    }

    public double getHeading() {
        return 0.0;
    }


    class Position{
        public double x = 0.0;
        public double y = 0.0;

        public void reset(){
            x = 0.0;
            y = 0.0;
        }


        public void move(double distance, double angle){
            double deltaX = Math.cos(angle)*distance;
            double deltaY = Math.sin(angle)*distance;

            change(deltaX,deltaY);
        }

        public void change(double deltaX, double deltaY){
            x += deltaX;
            y += deltaY;
        }
    }
}
