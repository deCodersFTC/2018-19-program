/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="deCoders AutStop", group="Linear Opmode")
public class AutonomousHookDescent extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor  LeftDriveFront = null;
    public DcMotor  RightDriveFront = null;
    public DcMotor  LeftDriveBack = null;
    public DcMotor  RightDriveBack = null;

    public DcMotor  lift = null;

    public DistanceSensor heightSensor = null;

    public ColorSensor sensorColor = null;
    public DistanceSensor sensorDistance = null;

    BNO055IMU imu;


    //create variables for encoders
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 1;
    static final double TURN_SPEED = 0.7;


    // Margin of error values
    static final double MOE_ANGLE = 0.05;
    static final double MOE_HEIGHT = 0.01;

    //declare robot with pushbot hardware
    private HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware


    // State used for updating IMU telemetry
    Orientation angles;
    Orientation autStep2Angles;
    Orientation autStep3Angles;
    Acceleration gravity;
    public boolean opStat;
    public String opStatusString;
    boolean whileStep2;
    int whileScanning;


    public boolean ifStopped() {
        if (!opModeIsActive()) {
            opStat = false;
            stopWheels();
            return true;
        }
        return false;
    }
    public void initBot() {
        robot.init(hardwareMap,LeftDriveFront, RightDriveFront, LeftDriveBack, RightDriveBack, lift, heightSensor, sensorColor, sensorDistance, imu );
        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);



        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();

    }





    //------------------------------------------------------------------------------------
    // Motor controls
    //------------------------------------------------------------------------------------
    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed, double LeftFrontInches, double RightFrontInches, double LeftBackInches, double RightBackInches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            initBot();
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = LeftDriveFront.getCurrentPosition() + (int) (LeftFrontInches * COUNTS_PER_INCH);
            newRightFrontTarget = RightDriveFront.getCurrentPosition() + (int) (RightFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = LeftDriveBack.getCurrentPosition() + (int) (LeftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = RightDriveBack.getCurrentPosition() + (int) (RightBackInches * COUNTS_PER_INCH);
            LeftDriveFront.setTargetPosition(newLeftFrontTarget);
            RightDriveFront.setTargetPosition(newRightFrontTarget);
            LeftDriveBack.setTargetPosition(newLeftBackTarget);
            RightDriveBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            LeftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            LeftDriveFront.setPower(Math.abs(speed));
            RightDriveFront.setPower(Math.abs(speed));
            LeftDriveBack.setPower(Math.abs(speed));
            RightDriveBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (LeftDriveFront.isBusy() && RightDriveFront.isBusy() && LeftDriveBack.isBusy() && RightDriveBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :7d :7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :7d :7d",
                        LeftDriveFront.getCurrentPosition(),
                        RightDriveFront.getCurrentPosition(),
                        LeftDriveBack.getCurrentPosition(),
                        RightDriveBack.getCurrentPosition());
                telemetry.update();

            }


            stopWheels();
        }
    }

    // the until condition interface allows for the autonomous system to execute an operation
    // until the condition is met.
    public interface UntilCondition {
        public boolean until();
    }

    public void stopWheels() {
        LeftDriveFront.setPower(0);
        RightDriveFront.setPower(0);
        LeftDriveBack.setPower(0);
        RightDriveBack.setPower(0);

        // Turn off RUN_TO_POSITION
        LeftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveForward(double distance) {
        encoderDrive(DRIVE_SPEED, -distance, -distance, -distance, -distance, 5.0);
    }

    /**
     * Drive robot at a given speed ratio
     *
     * @param speedRatio 1 is fastest, 0 is stopped, -1 goes backwards
     */
    public void drive(float speedRatio) {

        if (speedRatio < -1 || speedRatio > 1) {
//            Log.e("Invalid speed value:" + speedRatio);
            return;
        }

        LeftDriveFront.setPower(speedRatio);
        RightDriveFront.setPower(-speedRatio);
        LeftDriveBack.setPower(-speedRatio);
        RightDriveBack.setPower(speedRatio);
    }

    /**
     * Turns the bot given the ratio.
     *
     * @param turnRatio -1 turns 90 deg left, 0=straight, 1=turn 90 deg right
     */
    public void turn(float turnRatio) {
        //-1 turns right, 0 straight, 1 turns left, anything else does nothing
        if (turnRatio < -1 || turnRatio > 1) {
//            Log.e("Invalid turn value:" + turnRatio);
            return;
        }

        double deg = turnRatio * 90;
        if (deg < 0) {
            turnLeft(deg);
        } else {
            turnRight(deg);
        }
    }

    /**
     * Slide robot at a given speed ratio
     *
     * @param slideRatio 1 is right, 0 is stopped, -1 goes left
     */
    public void slide(float slideRatio) {
        if (slideRatio < -1 || slideRatio > 1) {
//            Log.e("Invalid speed value:" + speedRatio);
            return;
        }

        LeftDriveFront.setPower(-slideRatio);
        RightDriveFront.setPower(slideRatio);
        LeftDriveBack.setPower(slideRatio);
        RightDriveBack.setPower(-slideRatio);
    }


    public void driveBackward(double distance) {
        encoderDrive(DRIVE_SPEED, distance, distance, distance, distance, 5.0);
    }

    public void slideRight(double distance) {
        encoderDrive(DRIVE_SPEED, distance, -distance, -distance, distance, 5.0);
    }

    public void slideLeft(double distance) {
        encoderDrive(DRIVE_SPEED, -distance, distance, distance, -distance, 5.0);
    }

    public void turnRight(double a) {
        double degrees = a * 24 / 90;
        encoderDrive(DRIVE_SPEED, -degrees, degrees, -degrees, degrees, 5.0);
    }

    public void turnLeft(double a) {
        double degrees = a * 24 / 90;
        encoderDrive(DRIVE_SPEED, degrees, -degrees, degrees, -degrees, 5.0);
    }

    public void driveForwardUntil(UntilCondition u) {
        // record heading from IMU
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float origHeading = angles.firstAngle;

        telemetry.addData("Current Heading:", origHeading);

        while (u.until()) {
            if (!ifStopped()) {
                float adjHeading = imu.getAngularOrientation().firstAngle - origHeading;
                if ((adjHeading * adjHeading) < MOE_ANGLE) {
                    // drive forward keeping heading constant
                    drive(1);
                } else {
                    turn(adjHeading);
                }
            }
        }
        stopWheels();
    }
    public void BlockSensing(){

        whileScanning = 1;
        while(whileScanning <= 3){
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());

            if (sensorColor.red() == 255 && sensorColor.green() == 255 && sensorColor.blue() == 0){
                driveForward(10);
            }
            else{
                slideLeft(2.8);
            }
            whileScanning++;
        }

    }

    public void slideRightStep2() {

        whileStep2 = true;

        autStep2Angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float origRecord = autStep2Angles.firstAngle;
        telemetry.addData("Current Heading:", origRecord);

        while (whileStep2) {

            if (!ifStopped()) {
                float devHeading = imu.getAngularOrientation().firstAngle - origRecord;
                while (whileStep2) {
                    telemetry.addData("Current Deviation", devHeading);
                    telemetry.update();
                }
                if ((devHeading * devHeading) < MOE_ANGLE) {
                    // drive forward keeping heading constant
                    slideRight(3);
                } else {
                    turn(-devHeading);
                }
            }
        }
        whileStep2 = false;
    }

    public void driveForwardStep3() {

        boolean whileStep3 = true;

        autStep3Angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float origRecord1 = autStep3Angles.firstAngle;
        telemetry.addData("Current Heading:", origRecord1);

        while (whileStep3) {

            if (!ifStopped()) {
                float driveHeading = imu.getAngularOrientation().firstAngle - origRecord1;
                while (whileStep3) {
                    telemetry.addData("Current Deviation", driveHeading);
                    telemetry.update();
                }
                if ((driveHeading * driveHeading) < MOE_ANGLE) {
                    // drive forward keeping heading constant
                    driveForward(42);
                } else {
                    turn(-driveHeading);
                }
            }
        }
        whileStep2 = false;
    }

    public void slideRightUntil(UntilCondition u) {
        while (u.until()) {
            if (!ifStopped()) {
                // drive forward keeping heading constant
                drive(1);
            }
        }
        stopWheels();
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    private void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });
    }

    private void lowerHook(UntilCondition u) {
        while (u.until()) {

        }
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }




    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initBot();

        waitForStart();
        runtime.reset();

//        lowerHook(new UntilCondition() {
//            @Override
//            public boolean until() {
//                double x = heightSensor.getDistance(DistanceUnit.INCH) - robot.ZERO_HEIGHT;
//                return (x * x) < MOE_HEIGHT;
//            }
//        });

        // the following is the state of the autonomous mode
        // 1. Descent
        lowerHook(new UntilCondition() {
            @Override
            public boolean until() {
                initBot();
                double x = heightSensor.getDistance(DistanceUnit.INCH) - robot.ZERO_HEIGHT;
                if (((Math.abs(x)) < MOE_HEIGHT) == true) {
                    lift.setPower(1.0);
                } else {
                    lift.setPower(0);
                }
                return (x * x) < MOE_HEIGHT; }});
        telemetry.addData("Current Step", 1);


        // 2. Slide right
        slideRightStep2();
        telemetry.addData("Current Step", 2);


        // 3. Move forward 3.5 ft
        driveForwardStep3();
        telemetry.addData("Current Step", 3);
        // 4. Turn right
        turnRight(90);
        driveForward(30);
        turnRight(90);

        telemetry.addData("Current Step", 4);
        // 5. Scan for mineral using Color sensor and knock mineral by advancing bot
        BlockSensing();
        driveBackward(10);
        turnLeft(90);

        // 7. Go forward to the wall
       /* if (frontSensor < 30){
            driveForward(30);
        }else{
            //drop team marker
            stopWheels();
        }*/


        // 8. Move forward until the bot senses black color (crater)


    }
}