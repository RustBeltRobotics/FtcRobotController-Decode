/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * FINAL WORKING VERSION - Correct atan2 order
 */
package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.media.SoundPool;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.IOException;
import java.util.Locale;

@TeleOp(name = "!_SVPUtiliserCeModeTeleop2", group = "Robot")
public class SVPUtiliserCeModeTeleop2 extends LinearOpMode {
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor shooter;
    DcMotor intake;
    DcMotor feeder;
    DcMotor feeder2;

    Rev9AxisImu imu;
    Orientation angles;

    WebTelemetryStreamer webTelemetryStreamer;
    WebInterface webInterface;

    boolean shooterToggle = false;
    boolean lastGamepadX = false;

    Toggler aToggler = new Toggler();
    DriveController driveController;

    double yawZero;

    @Override
    public void runOpMode() {
        Rev9AxisImu.Parameters parameters = new Rev9AxisImu.Parameters(
                new Rev9AxisImuOrientationOnRobot(
                        Rev9AxisImuOrientationOnRobot.LogoFacingDirection.UP,
                        Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.BACKWARD
                )
        );

        imu = hardwareMap.get(Rev9AxisImu.class, "external_imu");
        imu.initialize(parameters);

        webTelemetryStreamer = new WebTelemetryStreamer(8886);
        Thread webTelemetryStreamerThread = new Thread(webTelemetryStreamer);
        webTelemetryStreamerThread.start();

        webInterface = new WebInterface(8885);
        webInterface.addParameter("shooter_power", 0.55);
        webInterface.addParameter("feeder2_power", 0.4);
        webInterface.addParameter("Kp_drive", 0.01);
        webInterface.addParameter("Ki_drive", 0.00);
        webInterface.addParameter("Kd_drive", 0.00);
        webInterface.addParameter("dbsizec", 0.3);
        webInterface.addParameter("dbdepthc", 0.3);

        Thread webInterfaceThread = new Thread(webInterface);
        webInterfaceThread.start();

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        feeder2 = hardwareMap.get(DcMotor.class, "feeder2");

        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        feeder.setDirection(DcMotor.Direction.FORWARD);
        feeder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        feeder2.setDirection(DcMotor.Direction.FORWARD);
        feeder2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        driveController = new DriveController(imu, frontRightDrive, frontLeftDrive, backLeftDrive, backRightDrive, new PIDController(0.0, 0.0, 0.0));
        driveController.init();

        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        driveController.yawZero = 0.0 - angles.firstAngle;

        int loopcounter = 0;
        while (opModeIsActive()) {
            loop2(loopcounter++);
        }

        try {
            webInterface.stop();
        } catch (IOException e) {}

        try {
            webTelemetryStreamer.stop();
        } catch (IOException e) {}
    }

    private double avg(double a, double b) {
        return (a + b) / 2;
    }

    private double boolToDoubleBecauseItWontCast(boolean input) {
        return input ? 1.0 : 0.0;
    }

    public void loop2(int loopcounter) {
        telemetry.addLine("Press DPAD UP to reset Yaw");
        telemetry.addLine("RIGHT BUMPER for robot-relative");

        driveController.drivingPID.setCoefs(
                webInterface.getParameter("Kp_drive"),
                webInterface.getParameter("Ki_drive"),
                webInterface.getParameter("Kd_drive")
        );

        driveController.drivingPID.setDeadbandStuff(
                webInterface.getParameter("dbsizec"),
                webInterface.getParameter("dbdepthc")
        );

        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));

        if (loopcounter % 3 == 0) {
            webTelemetryStreamer.sendData("heading", angles.firstAngle);
            webTelemetryStreamer.sendData("x", gamepad1.left_stick_x * 100.0);
            webTelemetryStreamer.sendData("y", gamepad1.left_stick_y * 100.0);

            webTelemetryStreamer.sendData("current_frontLeftDrive", ((DcMotorEx) frontLeftDrive).getCurrent(CurrentUnit.MILLIAMPS));
            webTelemetryStreamer.sendData("current_frontRightDrive", ((DcMotorEx) frontRightDrive).getCurrent(CurrentUnit.MILLIAMPS));
            webTelemetryStreamer.sendData("current_backLeftDrive", ((DcMotorEx) backLeftDrive).getCurrent(CurrentUnit.MILLIAMPS));
            webTelemetryStreamer.sendData("current_backRightDrive", ((DcMotorEx) backRightDrive).getCurrent(CurrentUnit.MILLIAMPS));
        }

        double invert_all_emergency = ((Math.min((gamepad1.left_bumper ? 1 : 0) + (gamepad2.left_bumper ? 1 : 0), 1.0)) * 2 - 1);

        if (!gamepad1.right_bumper) {
            driveFieldRelative(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            telemetry.addLine("ROBOT RELATIVE MODE");
            drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        if (gamepad1.x && !lastGamepadX) {
            shooterToggle = !shooterToggle;
        }

        if (gamepad1.dpad_up) {
            driveController.yawZero = 0.0 - angles.firstAngle;
            telemetry.addLine(">>> YAW RESET <<<");
        }

        lastGamepadX = gamepad1.x;
        aToggler.update(gamepad1.a);

        double shooterPowerCoef = webInterface.getParameter("shooter_power");
        double upperWheelPower = webInterface.getParameter("feeder2_power");
        double restPowerLevel = 1.9;

        shooter.setPower(shooterToggle ? -shooterPowerCoef :
                (-invert_all_emergency * (((gamepad1.left_trigger * shooterPowerCoef - gamepad1.right_trigger * shooterPowerCoef) +
                        (gamepad2.left_trigger * shooterPowerCoef - gamepad2.right_trigger * shooterPowerCoef)) / 2.0)));

        intake.setPower(avg(gamepad2.left_stick_y * restPowerLevel,
                invert_all_emergency * boolToDoubleBecauseItWontCast(aToggler.currentState || gamepad1.b || gamepad1.y) * restPowerLevel));
        feeder.setPower(avg(gamepad2.right_stick_y * restPowerLevel,
                invert_all_emergency * boolToDoubleBecauseItWontCast(gamepad1.b || gamepad1.y) * restPowerLevel));
        feeder2.setPower(avg(gamepad2.right_stick_x * upperWheelPower,
                invert_all_emergency * -boolToDoubleBecauseItWontCast(gamepad1.y) * upperWheelPower));

        if (gamepad2.aWasPressed()) {
            webInterface.setParameter("shooter_power", 0.55);
        }
        if (gamepad2.bWasPressed()) {
            webInterface.setParameter("shooter_power", webInterface.getParameter("shooter_power") + 0.05);
        }
        if (gamepad2.xWasPressed()) {
            webInterface.setParameter("shooter_power", webInterface.getParameter("shooter_power") - 0.05);
        }

        telemetry.update();
    }

    // FINAL CORRECT FIELD-RELATIVE IMPLEMENTATION
    private void driveFieldRelative(double forward, double right, double rotate) {
        // Correct atan2 order (back to original)
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        double robotHeading = angles.firstAngle;
        double correctedHeading = AngleUnit.DEGREES.toRadians(robotHeading - this.yawZero);

        // Use + for heading correction
        theta = AngleUnit.normalizeRadians(theta + correctedHeading);

        // Standard polar to Cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // No swap - pass directly
        drive(newForward, newRight, rotate);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backRightPower), Math.abs(backLeftPower))
        );

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
            backLeftPower /= maxPower;
        }

        double maxSpeed = 1.0;

        frontLeftDrive.setPower(maxSpeed * frontLeftPower);
        frontRightDrive.setPower(maxSpeed * frontRightPower);
        backLeftDrive.setPower(maxSpeed * backLeftPower);
        backRightDrive.setPower(maxSpeed * backRightPower);
    }
}