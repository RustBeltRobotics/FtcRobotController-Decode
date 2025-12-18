package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorPIDFControlLoopCoefficientsCommand;
import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveController {

    public PIDController drivingPID;// = new PIDController(1.0, 0.1, 0.3); // meh pid

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    Rev9AxisImu imu;

    Orientation angles;

    double yawZero;

    DriveController(Rev9AxisImu imu, DcMotor frontRightDrive, DcMotor frontLeftDrive, DcMotor backLeftDrive, DcMotor backRightDrive, PIDController pid) {
        this.drivingPID = pid; // meh pid
        this.imu = imu;
        this.frontRightDrive = frontRightDrive;
        this.frontLeftDrive = frontLeftDrive;
        this.backLeftDrive = backLeftDrive;
        this.backRightDrive = backRightDrive;
    }

    void init() {
        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
//        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        ((DcMotorEx) frontLeftDrive).setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients());

        PIDFCoefficients currentCoefs = ((DcMotorEx) frontLeftDrive).getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        System.out.println(currentCoefs.toString());


        // original:
        //PIDFCoefficients coefs = new PIDFCoefficients(10.000000, 0.049988, 0.000000, 0.000000, LynxSetMotorPIDFControlLoopCoefficientsCommand.InternalMotorControlAlgorithm.LegacyPID.toExternal());

        PIDFCoefficients coefs = new PIDFCoefficients(10.000000, 0.049988, 0.020000, 0.000000, LynxSetMotorPIDFControlLoopCoefficientsCommand.InternalMotorControlAlgorithm.LegacyPID.toExternal());


        ((DcMotorEx) frontLeftDrive).setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, coefs);
        ((DcMotorEx) frontRightDrive).setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, coefs);
        ((DcMotorEx) backLeftDrive).setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, coefs);
        ((DcMotorEx) backLeftDrive).setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, coefs);


    }

    void stop() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    public void driveFieldRelativeAuto(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // TODO: Continuousalize this, or use quaternion maybe, because I think wrapping around is causing the issue that only appears half the time
        angles = this.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double yaw_corrected = ((((0.0 - angles.firstAngle) - this.yawZero) + 180.0)) % 360.0 - 180.0;

        double currentError = AngleUnit.normalizeRadians(rotate - yaw_corrected * (Math.PI/180));
        drivingPID.setTarget(0);
        rotate = drivingPID.loop(currentError);

        theta = AngleUnit.normalizeRadians(theta - (yaw_corrected) * (3.141592653589/180));//AngleUnit.n

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // This routine drives the robot field relative
    public void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // TODO: Continuousalize this, or use quaternion maybe, because I think wrapping around is causing the issue that only appears half the time
        angles = this.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double yaw_corrected = ((((0.0 - angles.firstAngle) - this.yawZero) + 180.0)) % 360.0 - 180.0;

//        double currentError = AngleUnit.normalizeRadians(rotate - yaw_corrected * (Math.PI/180));
//        drivingPID.setTarget(0);
//        rotate = drivingPID.loop(currentError);

        theta = AngleUnit.normalizeRadians(theta - (yaw_corrected) * (3.141592653589/180));//AngleUnit.n

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        double coef = 900.0;

        frontLeftDrive.setPower(1.0);
        frontRightDrive.setPower(1.0);
        backLeftDrive.setPower(1.0);
        backRightDrive.setPower(1.0);

        int euc = (int) (coef * maxSpeed * (frontLeftPower / maxPower));
//        System.out.print("test: ");
//        System.out.println(euc);
//        System.out.println(frontLeftPower);
//        System.out.println(forward);


        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + (int) (coef * maxSpeed * (frontLeftPower / maxPower)));
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + (int) (coef * maxSpeed * (frontRightPower / maxPower)));
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() + (int) (coef * maxSpeed * (backLeftPower / maxPower)));
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + (int) (coef * maxSpeed * (backRightPower / maxPower)));

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
