package org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFileAfterWV.Auto;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFileAfterWV.RelicRecoveryResources.RelicRecoveryConfigV2Cleve;

/**
 * Created by Brandon on 12/5/2017.
 */

public class RelicRecoveryAutonomousMainV3 extends RelicRecoveryConfigV2Cleve {

    public enum Auto{
        SCAN, JEWELS, DRIVEOFFPLATFROM, TURN, LINEUPTOCRYPTO, PUTGLYPH, PREPARECOLLECT,COLLECT,PREPAREDRIVEBACKTOCRYPTO,DRIVEBACKTOCRYPTO
    }

    ElapsedTime time = new ElapsedTime();

    Auto auto = Auto.SCAN;

    double facingCrypto = 0.0;

    Position pos = new Position();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.update();
        //load robot
        config(this);
        //load everything
        loadSwitchBoard();

        //wait for start
        telemetry.addLine("Waiting for Start");
        telemetry.addData("Time(milliseconds)",time.milliseconds());
        telemetry.update();
        waitForStart();

        time.reset();

        //testing
        auto = Auto.PREPARECOLLECT;

        while (opModeIsActive()) {
            telemetry.addData("Time(milliseconds)",time.milliseconds());
            telemetry.addData("Running",auto);
            switch (colorPosition) {
                case REDFRONT:
                    switch (auto) {
                        case SCAN:
                            //TODO scan
                            break;
                        case JEWELS:
                            //TODO jewels
                            break;
                        case DRIVEOFFPLATFROM:
                            //drive off platform
                            //TODO add encoder drive at the end
                            if (driveOffPlatform(.5))
                                auto = Auto.TURN;
                            break;
                        case TURN:
                            //TODO turn
                            break;
                        case LINEUPTOCRYPTO:
                            //TODO line up to wall


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

                    break;
                case BLUEFRONT:

                    break;
                case BLUEBACK:

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
            driveOff = 2;
        } else if (driveOff == 3) {
            //TODO encoder drive then return true
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




