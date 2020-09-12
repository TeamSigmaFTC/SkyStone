package org.firstinspires.ftc.teamcode;

// import android.nfc.tech.TagTechnology;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp
public class MyFIRSTJavaOpMode extends LinearOpMode {
    //defines member fields
//    private Gyroscope imu;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle, power = .30, correction;
    private boolean aButton, bButton, touched;

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .05;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    @Override
    public void runOpMode() {
//        imu = hardwareMap.get(Gyroscope.class, "imu");
        //assigns motor to member fields
        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        backRight = hardwareMap.get(DcMotor.class, "BackRight");
        //resets encoders to zero
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set power ---> runs
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
        //waits for driver to press play
        waitForStart();
        // run until the end of the match (driver presses STOP)
        //defines local variables
        double x = 0;
        double y = 0;
        double rotation = 0;
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;
        double maxAbsPower;
        double maxPower = 0.5;

        //while running
        while (opModeIsActive()) {
            // Use gyro to drive in a straight line.
            correction = checkDirection();

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();

            // forward and backwards
            //assigns gamepads joysticks directions
            x = -this.gamepad1.left_stick_x;
            y = -this.gamepad1.left_stick_y;
            rotation = -this.gamepad1.right_stick_x;

            if (rotation == 0.0) {
                //corrects the robot to maintain its heading if no turning is needed
                rotation = correction;
            } else {
                resetAngle();
            }

            frontLeftPower = rotation - y - x;
            frontRightPower = rotation + y + x;
            backLeftPower = rotation - y + x;
            backRightPower = rotation + y - x;

            maxAbsPower = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            maxAbsPower = Math.max(Math.abs(backLeftPower), maxAbsPower);
            maxAbsPower = Math.max(Math.abs(backRightPower), maxAbsPower);
            if (maxAbsPower > 1) {
                //maximum power value becomes 1
                frontLeftPower = frontLeftPower / maxAbsPower;
                frontRightPower = frontRightPower / maxAbsPower;
                backLeftPower = backLeftPower / maxAbsPower;
                backRightPower = backRightPower / maxAbsPower;
            }

            frontLeftPower = frontLeftPower * maxPower;
            frontRightPower = frontRightPower * maxPower;
            backLeftPower = backLeftPower * maxPower;
            backRightPower = backRightPower * maxPower;

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            //logs for puny humans
            //sends power and position (degrees the wheels have spun) to driver station.
            telemetry.addData("frontLeftPower", (frontLeftPower));
            telemetry.addData("frontRightPower", (frontRightPower));
            telemetry.addData("backLeftPower", (backLeftPower));
            telemetry.addData("backRightPower", (backRightPower));
            telemetry.addData("frontLeft Position", frontLeft.getCurrentPosition());
            telemetry.addData("frontRight Position", frontRight.getCurrentPosition());
            telemetry.addData("backLeft Position", backLeft.getCurrentPosition());
            telemetry.addData("backRight Position", backRight.getCurrentPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}

