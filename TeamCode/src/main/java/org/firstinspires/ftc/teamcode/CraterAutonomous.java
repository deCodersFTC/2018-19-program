public void encoderLift(){
         angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float angle_at_top = angles.firstAngle;
        lift.setPower(1.0);
        sleep(8000);
        telemetry.addData("Landed", "ground?");
        telemetry.update();
        lift.setPower(0);
        Backwards(10);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        AccurateTurn(angles.firstAngle - angle_at_top);
        slideRight(10);
        
    }
