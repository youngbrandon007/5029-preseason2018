package org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleConfigLinearOpMode;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleMotor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleRobot;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleServo;

/**
 * Created by young on 9/14/2017.
 */

abstract public class RelicRecoveryConfigV2 extends PineappleConfigLinearOpMode {

    public PineappleMotor driveFrontRight;
    public PineappleMotor driveFrontLeft;
    public PineappleMotor driveBackRight;
    public PineappleMotor driveBackLeft;
    public PineappleMotor conveyRight;
    public PineappleMotor conveyLeft;

    public PineappleServo phoneTurnLeft;
    public PineappleServo jewelRotationLeft;
    public PineappleServo jewelLeverLeft;
    public PineappleServo collectorLeft;
    public PineappleServo phoneTurnRight;
    public PineappleServo jewelRotationRight;
    public PineappleServo jewelLeverRight;
    public PineappleServo collectorRight;
    public PineappleServo conveyorFlipRight;
    public PineappleServo conveyorFlipLeft;

    public final int NAVX_DIM_I2C_PORT = 0;
    public AHRS navx_device;
    public final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    public PineappleEnum.AllianceColor allianceColor = PineappleEnum.AllianceColor.BLUE;
    @Override
    public void config(LinearOpMode linearOpMode) {
        robotHandler = new PineappleRobot(linearOpMode);


        driveFrontRight = robotHandler.motorHandler.newDriveMotor("FR", 1, false, false, PineappleEnum.MotorLoc.RIGHTFRONT, PineappleEnum.MotorType.NEV40);
        driveFrontLeft = robotHandler.motorHandler.newDriveMotor("FL", 1, false, false, PineappleEnum.MotorLoc.LEFTFRONT, PineappleEnum.MotorType.NEV40);
        driveBackRight = robotHandler.motorHandler.newDriveMotor("BR", 1, false, false, PineappleEnum.MotorLoc.RIGHTBACK, PineappleEnum.MotorType.NEV40);
        driveBackLeft = robotHandler.motorHandler.newDriveMotor("BL", 1, false, false, PineappleEnum.MotorLoc.LEFTBACK, PineappleEnum.MotorType.NEV40);
        conveyLeft = robotHandler.motorHandler.newDriveMotor("ConL", 1, false, false, PineappleEnum.MotorLoc.NONE, PineappleEnum.MotorType.NEV40);
        conveyRight = robotHandler.motorHandler.newDriveMotor("ConR", 1, false, false, PineappleEnum.MotorLoc.NONE, PineappleEnum.MotorType.NEV40);

        phoneTurnLeft = robotHandler.servoHandler.newLimitServo("PTL", 1, RelicRecoveryConstants.PHONELEFTFLAT);
        jewelRotationLeft = robotHandler.servoHandler.newLimitServo("JRL", 1, RelicRecoveryConstants.JEWELLEFTTURNLEFT);
        jewelLeverLeft = robotHandler.servoHandler.newLimitServo("JLL", 1, RelicRecoveryConstants.JEWELLEFTUP);
        collectorLeft = robotHandler.servoHandler.newContinuousServo("CL", 0.5);

        phoneTurnRight = robotHandler.servoHandler.newLimitServo("PTR", 1, RelicRecoveryConstants.PHONERIGHTFLAT);
        jewelRotationRight = robotHandler.servoHandler.newLimitServo("JRR", 1, RelicRecoveryConstants.JEWELRIGHTTURNRIGHT);
        jewelLeverRight = robotHandler.servoHandler.newLimitServo("JLR", 1, RelicRecoveryConstants.JEWELRIGHTUP);
        collectorRight = robotHandler.servoHandler.newContinuousServo("CR", 0.5);

        conveyorFlipLeft = robotHandler.servoHandler.newLimitServo("CFL", 1, RelicRecoveryConstants.FLIPINLEFT);
        conveyorFlipRight = robotHandler.servoHandler.newLimitServo("CFR", 1, RelicRecoveryConstants.FLIPINRIGHT);
        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        boolean calibration_complete = false;
        while ( !calibration_complete&&!opModeIsActive()&&!isStopRequested()) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            calibration_complete = !navx_device.isCalibrating();
            if (!calibration_complete) {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
                telemetry.update();
            }
        }
        telemetry.addData("navX-Micro", "Calibration Finished");
        telemetry.update();
        navx_device.zeroYaw();
    }



}
