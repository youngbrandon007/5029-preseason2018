
package org.firstinspires.ftc.teamcode.Old_Robots.NerfVuforiaRobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

/**
 * Created by Brandon on 3/31/2017.
 */

@TeleOp(name = "VuforiaTarget", group = "Linear Opmode")
@Disabled
public class VuforiaOpModeTargetTest extends com.qualcomm.robotcore.eventloop.opmode.LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    double maxSpeedH = .03;
    double maxSpeedUp = .06;
    double maxSpeedDown = .03;

    DcMotor Rotate;
    DcMotor Tilt;

    Servo Trigger;

    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AdB8VB7/////AAAAGcfBp9I80URFkfBQFUyM+ptmQXBAMGx0svJKz7QE2nm20mBc/zI5sZNHfuP/ziIm+sYnO7fvPqUbFs8QWjRyXVEDmW4mMj+S+l+yaYRkpGZ/pmHyXiDb4aemHx0m70BulMNIce4+NVaCW5S/5BWNNev/AU0P+uWnHYuKNWzD2dPaRuprC4R6b/DgD1zeio1xlssYb9in9mfzn76gChOrE5B0ql6Q9FiHC5cTdacq2lKjm5nlkTiXz1e2jhVK3SddGoqM4FQ3mFks7/A88hFzlPfIIk45K2Lh7GvcVjuIiqNj5mTLaZJVqlsKdTQnKS4trJcc1YV9sjdbmh1agtn1UePy91fDj9uWSBdXvpIowv4B";
        params.cameraMonitorFeedback =  VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,1);

        Rotate = hardwareMap.dcMotor.get("r");
        Tilt = hardwareMap.dcMotor.get("t");

        Rotate.setDirection(DcMotorSimple.Direction.REVERSE);
        Tilt.setDirection(DcMotorSimple.Direction.REVERSE);

        Trigger = hardwareMap.servo.get("tr");

        Trigger.setPosition(1);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("Target");
        beacons.get(0).setName("target");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        beacons.activate();

        double calcDegrees = 0;
        double calcDegreesV = 0;
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run : " + runtime.toString());

            for(VuforiaTrackable beac : beacons){
                OpenGLMatrix pose =  ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();
                if(pose!=null){
                    VectorF translation = pose.getTranslation();
                    //telemetry.addData(beac.getName() + " Translation", translation);
                    //if horizontal change the 1 & 2 to 0 & 2
                    double degreesToTurn =  Math.toDegrees(Math.atan2(translation.get(1),translation.get(2)));

                    //telemetry.addData(beac.getName() + "-Degrees Horizontal", degreesToTurn);

                    calcDegrees = 180 - Math.abs(degreesToTurn);

                    if(degreesToTurn<0){
                        calcDegrees= 0-calcDegrees;
                    }

                    telemetry.addData(beac.getName() + "-Calculated Degrees Horizontal",calcDegrees);

                    double rotateSpeed = Range.clip(calcDegrees/300, -maxSpeedH,maxSpeedH);

                    Rotate.setPower(rotateSpeed);

                    double degreesToTurnV =  Math.toDegrees(Math.atan2(translation.get(0),translation.get(2)));

                    //telemetry.addData(beac.getName() + "-Degrees Vertical", degreesToTurnV);

                    calcDegreesV = 180 - Math.abs(degreesToTurnV);

                    if(degreesToTurnV<0){
                        calcDegreesV= 0-calcDegreesV;
                    }

                    telemetry.addData(beac.getName() + "-Calculated Degrees Vertical",calcDegreesV);

                    double tiltSpeed = Range.clip(calcDegreesV/300,-maxSpeedUp, maxSpeedDown);

                    Tilt.setPower(tiltSpeed);

                }else{
                    Rotate.setPower(0);
                    Tilt.setPower(-.05);
                }
                if (gamepad1.a && gamepad1.b){
                    Trigger.setPosition(0);
                }
                else{
                    Trigger.setPosition(1);
                }

            }
            telemetry.update();



        }
    }

}

