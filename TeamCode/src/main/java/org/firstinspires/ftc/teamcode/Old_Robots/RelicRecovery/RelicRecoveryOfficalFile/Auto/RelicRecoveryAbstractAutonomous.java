package org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.Auto;

import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraCalibration;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.Tool;
import com.vuforia.Vec3F;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources.RelicRecoveryConstants;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleServo;
import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources.RelicRecoveryConfigV2;
import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources.RelicRecoveryEnums;

import java.text.DecimalFormat;
import java.util.Arrays;


/**
 * Created by young on 9/15/2017.
 */

abstract public class RelicRecoveryAbstractAutonomous extends RelicRecoveryConfigV2 {
    public double delay = 0;
    public RelicRecoveryEnums.AutoColor color = RelicRecoveryEnums.AutoColor.BLUE;
    public RelicRecoveryEnums.StartingPosition position = RelicRecoveryEnums.StartingPosition.FRONT;
    public RelicRecoveryEnums.ColorPosition colorPosition = RelicRecoveryEnums.ColorPosition.BLUEFRONT;
    public boolean moreGlyph = false;
    public boolean gyroEnabled = true;
    public boolean glyphsEnabled = true;
    public boolean delayEnabled = true;
    public boolean pidEnabled = false;
    public boolean encoderEnabled = false;
    public boolean jewelsEnabled = true;
    public boolean vuforiaAlign = true;
    public boolean colorAlign = false;

    private boolean usingGyro = false;

    public double lastPosition = .75;


    public void updateServoVuforia(VuforiaTrackableDefaultListener listener, PineappleServo servo) {

    }

    public void alignToCrypto(double angle, VuforiaTrackableDefaultListener listener, VectorF vector) {
        final double TARGET_ANGLE_DEGREES = angle;
        final double TOLERANCE_DEGREES = 2.0;

        double YAW_PID_P = RelicRecoveryConstants.ROBOTTURNP;
        double YAW_PID_I = RelicRecoveryConstants.ROBOTTURNI;
        double YAW_PID_D = RelicRecoveryConstants.ROBOTTURND;
        navXPIDController xpidController = new navXPIDController(navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        xpidController.setSetpoint(TARGET_ANGLE_DEGREES);
        xpidController.setContinuous(true);
        xpidController.setOutputRange(RelicRecoveryConstants.MIN_MOTOR_OUTPUT_VALUE, RelicRecoveryConstants.MAX_MOTOR_OUTPUT_VALUE);
        xpidController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        xpidController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        try {
            xpidController.enable(true);

            int DEVICE_TIMEOUT_MS = 1000;
            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

            DecimalFormat df = new DecimalFormat("#.###");
            boolean done = false;
            while (opModeIsActive() && !Thread.currentThread().isInterrupted() && !done) {

                double output = 0;
                if (xpidController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        telemetry.addData("PIDOutput", df.format(0.00));
                    } else {
                        output = yawPIDResult.getOutput();
                        telemetry.addData("PIDOutput", df.format(output) + ", " +
                                df.format(-output));
                    }
                } else {
                /* A timeout occurred */
                    telemetry.addData("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
                telemetry.addData("Yaw", df.format(navx_device.getYaw()));
                telemetry.update();


                done = alignToCryptoboxVuforia(listener, vector, output);
            }

        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();

        }
    }


    //Vuforia Functions
    private boolean alignToCryptoboxVuforia(VuforiaTrackableDefaultListener listener, VectorF vector, double rotation) {
        if (null != listener.getPose()) {
            VectorF trans = listener.getPose().getTranslation();
            // Extract the X, Y, and Z components of the offset of the target relative to the robot
            double[] robot = {trans.get(1), trans.get(2)};

            double[] target = {robot[0] - vector.get(1), robot[1] - vector.get(2)};//Delta change till target


            telemetry.addData("x", robot[0]); // X
            telemetry.addData("y", robot[1]); // Y
            telemetry.addData("-x", target[0]);
            telemetry.addData("-y", target[1]);


            double dis = getDistance(target);
            double ang = getAngle(target) - 45;
            ang += 90;
            telemetry.addData("Distance", dis);
            telemetry.addData("Angle", ang);

            //Check for stop before motors are set again
            double gyroAngle = navx_device.getYaw();
            gyroAngle += (gyroAngle < 0) ? 360 : 0;

            if (dis < RelicRecoveryConstants.VUFORIAALIGNRANGE && inRange(gyroAngle, 2, 358)) {
                robotHandler.drive.stop();
                return true;
            }

            double speed = (dis > RelicRecoveryConstants.VUFORIAALIGNMEDIUM) ? 0.7 : (dis > RelicRecoveryConstants.VUFORIAALIGNSLOW) ? .4: .2;

            robotHandler.drive.mecanum.setMecanum(Math.toRadians(ang), speed, rotation, 1);


        }
        return false;
    }

    private boolean inRange(double value, double lowerLimit, double upperLimit) {
        return value > lowerLimit && value < upperLimit;
    }

    private double getDistance(double[] target) {
        return Math.sqrt(Math.pow(target[0], 2) + Math.pow(target[1], 2));
    }

    private double getAngle(double[] target) {
        double angle = (float) Math.toDegrees(Math.atan2(target[1], target[0]));

        angle += (angle < 0) ? 360 : 0;

        return angle;
    }


    //aling with gyro
    public boolean alignWithGyro(double angle, double power) throws InterruptedException {
        Thread.sleep(10);
        double gyroAngle = this.navx_device.getYaw();

        telemetry.addData("Gyro Angle", gyroAngle);
        telemetry.addData("Target Angle", angle);
        telemetry.update();

        //make  the range between 0-360
        gyroAngle += (gyroAngle < 0) ? 360 : 0;

        //offset to the input angle - positive is clockwise
        gyroAngle -= angle;
        gyroAngle += (gyroAngle < 0) ? 360 : (gyroAngle > 360) ? -360 : 0;

        double rotation = (gyroAngle > 1 && gyroAngle < 180) ? -power : (gyroAngle < 359 && gyroAngle > 179) ? power : 0;


        if (rotation == 0) {
            if (usingGyro) {
                robotHandler.drive.stop();
                usingGyro = false;
            }
            return true;
        } else usingGyro = true;


        robotHandler.drive.tank.setPower(rotation, rotation);

        return false;
    }


    public void gyroTurnPID(double degrees) throws InterruptedException {

        final double TARGET_ANGLE_DEGREES = degrees;
        final double TOLERANCE_DEGREES = 1.0;

        double YAW_PID_P = RelicRecoveryConstants.ROBOTTURNP;
        double YAW_PID_I = RelicRecoveryConstants.ROBOTTURNI;
        double YAW_PID_D = RelicRecoveryConstants.ROBOTTURND;
        navXPIDController xpidController = new navXPIDController(navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        xpidController.setSetpoint(TARGET_ANGLE_DEGREES);
        xpidController.setContinuous(true);
        xpidController.setOutputRange(RelicRecoveryConstants.MIN_MOTOR_OUTPUT_VALUE, RelicRecoveryConstants.MAX_MOTOR_OUTPUT_VALUE);
        xpidController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        xpidController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        try {
            xpidController.enable(true);

            int DEVICE_TIMEOUT_MS = 1000;
            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

            DecimalFormat df = new DecimalFormat("#.###");
            boolean done = false;
            while (opModeIsActive() &&
                    !Thread.currentThread().isInterrupted() && !done) {
//                servoCorrectForPicture(phoneTurnLeft, PineappleRelicRecoveryVuforia.getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), track, vuforia.getCameraCalibration(), telemetry);
                double output = 0;
                if (xpidController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        telemetry.addData("PIDOutput", df.format(0.00));
                    } else {
                        output = yawPIDResult.getOutput();
                        robotHandler.drive.tank.setPower(output, output);

                        telemetry.addData("PIDOutput", df.format(output) + ", " +
                                df.format(-output));
                    }
                } else {
                /* A timeout occurred */
                    telemetry.addData("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }

                if (navx_device.getYaw() > degrees - 1 && navx_device.getYaw() < degrees + 1) {
                    done = true;
                }
                telemetry.addData("Yaw", df.format(navx_device.getYaw()));
                telemetry.update();

            }
        } catch (Exception e) {
            Thread.currentThread().interrupt();
        }


    }

    public static void servoCorrectForPicture(PineappleServo servo, Image img, VuforiaTrackableDefaultListener track, CameraCalibration camCal, Telemetry telemetry, double pos) {
        double servoStart = servo.servoObject.getPosition();

        OpenGLMatrix pose = track.getRawPose();
        if (pose != null) {
            if (pose != null && img != null && img.getPixels() != null) {
                Matrix34F rawPose = new Matrix34F();
                float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
                rawPose.setData(poseData);
                float[] picture;
                picture = Tool.projectPoint(camCal, rawPose, new Vec3F(0, 0, 0)).getData();


                int middle = img.getWidth() / 2;
                telemetry.addData("Width", img.getWidth());
                telemetry.addData("X, Y", picture[0] + ", " + picture[1]);
                servoStart += (picture[0] > middle + 300) ? pos : (picture[0] < middle - 300) ? -pos : 0;
                servo.setPosition(servoStart);

            }

        }
    }

    public void hitJewels(PineappleEnum.JewelState jewelState) throws InterruptedException {
        if (jewelState != PineappleEnum.JewelState.NON_NON && jewelsEnabled) {
            jewelLeverLeft.setPosition(RelicRecoveryConstants.JEWELLEFTDOWN);
            jewelLeverRight.setPosition(RelicRecoveryConstants.JEWELRIGHTDOWN);
            jewelRotationLeft.setPosition(RelicRecoveryConstants.JEWELLEFTTURNMIDDLE);
            jewelRotationRight.setPosition(RelicRecoveryConstants.JEWELRIGHTTURNMIDDLE);
            Thread.sleep(1000);
            switch (jewelState) {
                case BLUE_RED:
                    if (allianceColor == PineappleEnum.AllianceColor.BLUE) {
                        jewelRotationLeft.setPosition(RelicRecoveryConstants.JEWELLEFTTURNRIGHT);
                    } else {
                        jewelRotationRight.setPosition(RelicRecoveryConstants.JEWELRIGHTTURNLEFT);
                    }
                    break;
                case RED_BLUE:
                    if (allianceColor == PineappleEnum.AllianceColor.RED) {
                        jewelRotationRight.setPosition(RelicRecoveryConstants.JEWELRIGHTTURNRIGHT);
                    } else {
                        jewelRotationLeft.setPosition(RelicRecoveryConstants.JEWELLEFTTURNLEFT);
                    }
                    break;
            }
            Thread.sleep(700);
            jewelLeverLeft.setPosition(RelicRecoveryConstants.JEWELLEFTUP);
            jewelRotationLeft.setPosition(RelicRecoveryConstants.JEWELLEFTTURNLEFT);
            jewelLeverRight.setPosition(RelicRecoveryConstants.JEWELRIGHTUP);
            jewelRotationRight.setPosition(RelicRecoveryConstants.JEWELRIGHTTURNRIGHT);
            Thread.sleep(500);
        }
    }

    public void driveOffPlate(double speed, double timeout, Image img, VuforiaTrackableDefaultListener track, CameraCalibration camCal) throws InterruptedException {
        //

        robotHandler.drive.mecanum.setPower(-speed, speed);
        ElapsedTime el = new ElapsedTime();
        el.reset();
//        double position = phoneTurnLeft.servoObject.getPosition();
        while (driveTillTilt() && opModeIsActive() && !(el.milliseconds() < timeout)) {
            Thread.sleep(10);
//            RelicRecoveryAbstractAutonomous.servoCorrectForPicture(phoneTurnLeft, img, track, camCal, telemetry);

        }
        while (driveTillFlat() && opModeIsActive() && !(el.milliseconds() < timeout)) {
//            position-=0.004;
//            phoneTurnLeft.setPosition(position);
//            RelicRecoveryAbstractAutonomous.servoCorrectForPicture(phoneTurnLeft, img, track, camCal, telemetry);

            Thread.sleep(10);
        }

        if ((el.milliseconds() < timeout)) {
            Thread.sleep(1500);
        }
        robotHandler.drive.stop();

    }

    public boolean driveTillTilt() {
        double roll = navx_device.getRoll();
        telemetry.addData("Roll", roll);
//        telemetry.update();
        return (Math.abs(roll) < 5);
    }

    public boolean driveTillFlat() {
        double roll = navx_device.getRoll();
        telemetry.addData("Roll", roll);
//        telemetry.update();
        return (Math.abs(roll) > 1);
    }

    public static double roundToHalf(double d) {
        return Math.round(d * 2) / 2.0;
    }

    public void loadSwitchBoard() {
        delay = robotHandler.switchBoard.loadAnalog("delay") * 2;
        color = (robotHandler.switchBoard.loadDigital("color")) ? RelicRecoveryEnums.AutoColor.BLUE : RelicRecoveryEnums.AutoColor.RED;
        position = (robotHandler.switchBoard.loadDigital("position")) ? RelicRecoveryEnums.StartingPosition.FRONT : RelicRecoveryEnums.StartingPosition.BACK;
        jewelsEnabled = robotHandler.switchBoard.loadDigital("jewel");
        pidEnabled = robotHandler.switchBoard.loadDigital("pid");
        glyphsEnabled = robotHandler.switchBoard.loadDigital("glyph");
        delayEnabled = robotHandler.switchBoard.loadDigital("delayEnabled");
        delay = roundToHalf(delay);
        switch (color){

            case RED:
                switch (position){

                    case FRONT:
                        colorPosition = RelicRecoveryEnums.ColorPosition.REDFRONT;
                        break;
                    case BACK:
                        colorPosition = RelicRecoveryEnums.ColorPosition.REDBACK;
                        break;
                }
                break;
            case BLUE:
                switch (position){

                    case FRONT:
                        colorPosition = RelicRecoveryEnums.ColorPosition.BLUEFRONT;
                        break;
                    case BACK:
                        colorPosition = RelicRecoveryEnums.ColorPosition.BLUEBACK;
                        break;
                }
                break;
        }

        telemetry.addData("Delay", delay);
        telemetry.addData("DelayEnabled", delayEnabled);
        telemetry.addData("Color", color);
        telemetry.addData("Jewel", jewelsEnabled);
        telemetry.addData("PID", pidEnabled);
        telemetry.addData("Glyphs", glyphsEnabled);
        telemetry.addData("Position", position);
    }

    public void loadSwitchBoardLoop() throws InterruptedException {
        while (opModeIsActive()) {
            loadSwitchBoard();
            telemetry.update();
            Thread.sleep(100);
        }
    }

    public void beginningDelay() throws InterruptedException {
        if (delayEnabled) {
            Thread.sleep((long) delay * 1000);
        }
    }

    public void flipTopOut() throws InterruptedException {
        conveyorFlipLeft.setPosition(RelicRecoveryConstants.FLIPOUTLEFT);
        conveyorFlipRight.setPosition(RelicRecoveryConstants.FLIPOUTRIGHT);
        Thread.sleep(1000);
        conveyorFlipLeft.setPosition(RelicRecoveryConstants.FLIPINLEFT);
        conveyorFlipRight.setPosition(RelicRecoveryConstants.FLIPINRIGHT);

    }
}


