package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Mecanum Type", group="Linear Opmode")
//@Disabled
public class MecanumPrototype extends LinearOpMode {



    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftDriveFront = null;
    private DcMotor RightDriveFront = null;
    private DcMotor LeftDriveBack = null;
    private DcMotor RightDriveBack = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        LeftDriveFront  = hardwareMap.get(DcMotor.class, "LeftDriveFront");
        RightDriveFront = hardwareMap.get(DcMotor.class, "RightDriveFront");
        LeftDriveBack  = hardwareMap.get(DcMotor.class, "LeftDriveBack");
        RightDriveBack = hardwareMap.get(DcMotor.class, "RightDriveBack");

        LeftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        RightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        LeftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        RightDriveBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double LeftFrontPower;
            double RightFrontPower;
            double RightBackPower;
            double LeftBackPower;

            double drive = gamepad1.left_stick_y;
            double slide = gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;


            LeftFrontPower = drive + slide + turn;
            RightFrontPower = drive - slide - turn;
            LeftBackPower = drive - slide + turn;
            RightBackPower = drive + slide - turn;

            if(turn != 0 && (drive != 0 || slide != 0)){
                LeftDriveFront.setPower(LeftFrontPower/2);
                LeftDriveBack.setPower(LeftBackPower/2);
                RightDriveFront.setPower(RightFrontPower/2);
                RightDriveBack.setPower(RightBackPower/2);

            }
            else{
                LeftDriveFront.setPower(LeftFrontPower);
                LeftDriveBack.setPower(LeftBackPower);
                RightDriveFront.setPower(RightFrontPower);
                RightDriveBack.setPower(RightBackPower);
            }
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors","Left Front Power: (%.2f), Right Front Power: (%.2f), Left Back Power: (%.2f), Right Back Power: (%.2f)", LeftDriveFront.getPower(), RightDriveFront.getPower(), LeftDriveBack.getPower(), RightDriveBack.getPower());
        telemetry.addData("Written by", "Arul and Samarth :)");
        telemetry.update();

    }
}