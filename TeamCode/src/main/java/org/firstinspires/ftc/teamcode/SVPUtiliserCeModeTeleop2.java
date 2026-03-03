/* Copyright (c) 2025 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.IOException;
import java.util.Locale;

/*
 * This OpMode illustrates how to program your robot to drive field relative.
 *
 * FIXED: Corrected field-relative math - changed minus to plus in theta calculation
 */
@TeleOp(name = "Robot: Field Relative FIXED", group = "Robot")
public class SVPUtiliserCeModeTeleop2 extends LinearOpMode {
    // This declares the four motors needed
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

    double yawZero;

    @Override
    public void runOpMode() {
        Rev9AxisImu.Parameters parameters = new Rev9AxisImu.Parameters(new Rev9AxisImuOrientationOnRobot(Rev9AxisImuOrientationOnRobot.LogoFacingDirection.UP, Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.BACKWARD));

        imu = hardwareMap.get(Rev9AxisImu.class, "external_imu");
        imu.initialize(parameters);

        System.out.println("MAIN ACTUAL OPCODE YAY A");

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        feeder2 = hardwareMap.get(DcMotor.class, "feeder2");

        webTelemetryStreamer = new WebTelemetryStreamer(8886);
        Thread webTelemetryStreamerThread = new Thread(webTelemetryStreamer);
        webTelemetryStreamerThread.start();

        webInterface = new WebInterface(8885);
        webInterface.addParameter("shooter_power", 0.551);
        webInterface.addParameter("feeder2_power", 0.4);
        webInterface.addParameter("Kp_drive", 0.01);
        webInterface.addParameter("Ki_drive", 0.00);
        webInterface.addParameter("Kd_drive", 0.00);

        PIDFCoefficients defaults_shooter =((DcMotorEx) shooter).getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        defaults_shooter.p = 90;

        webInterface.addParameter("Kp_shooter", defaults_shooter.p);
        webInterface.addParameter("Ki_shooter", defaults_shooter.i);
        webInterface.addParameter("Kd_shooter", defaults_shooter.d);
        webInterface.addParameter("Kf_shooter", defaults_shooter.f);

        Thread webInterfaceThread = new Thread(webInterface);
        webInterfaceThread.start();

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

        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        yawZero = 0.0 - angles.firstAngle;

        int loopcounter = 0;
        while (opModeIsActive()) {
            loop2(loopcounter++);
        }

        webInterface.stop();

        try {
            webTelemetryStreamer.stop();
        } catch (IOException e) {}
    }

    private double avg(double a, double b) {
        return (a + b)/2;
    }

    private double boolToDoubleBecauseItWontCast(boolean input) {
        return input ? 1.0 : 0.0;
    }

    public void loop2(int loopcounter) {
        telemetry.addLine("Press D-Pad UP to reset field-relative orientation");
        telemetry.addLine("Hold right bumper for robot-relative mode");
        telemetry.addLine("Left joystick controls direction");
        telemetry.addLine("Right joystick left/right rotates robot");

        ((DcMotorEx) shooter).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(
                        webInterface.getParameter("Kp_shooter"),
                        webInterface.getParameter("Ki_shooter"),
                        webInterface.getParameter("Kd_shooter"),
                        webInterface.getParameter("Kf_shooter")
                )
        );

        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("connection", imu.getConnectionInfo());
        telemetry.addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.addData("roll", formatAngle(angles.angleUnit, angles.secondAngle));
        telemetry.addData("pitch", formatAngle(angles.angleUnit, angles.thirdAngle));

        if (loopcounter % 3 == 0) {
            webTelemetryStreamer.sendData("heading", angles.firstAngle);
            webTelemetryStreamer.sendData("roll", angles.secondAngle);
            webTelemetryStreamer.sendData("pitch", angles.thirdAngle);

            webTelemetryStreamer.sendData("x", gamepad1.left_stick_x * 100.0);
            webTelemetryStreamer.sendData("y", gamepad1.left_stick_y * 100.0);
            webTelemetryStreamer.sendData("theta", gamepad1.right_stick_x * 100.0);

            webTelemetryStreamer.sendData("current_frontLeftDrive", ((DcMotorEx) frontLeftDrive).getCurrent(CurrentUnit.MILLIAMPS));
            webTelemetryStreamer.sendData("current_frontRightDrive", ((DcMotorEx) frontRightDrive).getCurrent(CurrentUnit.MILLIAMPS));
            webTelemetryStreamer.sendData("current_backLeftDrive", ((DcMotorEx) backLeftDrive).getCurrent(CurrentUnit.MILLIAMPS));
            webTelemetryStreamer.sendData("current_backRightDrive", ((DcMotorEx) backRightDrive).getCurrent(CurrentUnit.MILLIAMPS));

            webTelemetryStreamer.sendData("speed_shooter", -((DcMotorEx) shooter).getVelocity(AngleUnit.DEGREES));
            webTelemetryStreamer.sendData("speed_intake", -((DcMotorEx) intake).getVelocity(AngleUnit.DEGREES));
            webTelemetryStreamer.sendData("speed_feeder", -((DcMotorEx) feeder).getVelocity(AngleUnit.DEGREES));
            webTelemetryStreamer.sendData("speed_feeder2", ((DcMotorEx) feeder2).getVelocity(AngleUnit.DEGREES));

            webTelemetryStreamer.sendData("current_shooter", ((DcMotorEx) shooter).getCurrent(CurrentUnit.MILLIAMPS));
            webTelemetryStreamer.sendData("current_intake", ((DcMotorEx) intake).getCurrent(CurrentUnit.MILLIAMPS));
            webTelemetryStreamer.sendData("current_feeder", ((DcMotorEx) feeder).getCurrent(CurrentUnit.MILLIAMPS));
            webTelemetryStreamer.sendData("current_feeder2", ((DcMotorEx) feeder2).getCurrent(CurrentUnit.MILLIAMPS));

            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                break;
            }
        }

        double invert_all_emergency = ((Math.min((gamepad1.left_bumper ? 1 : 0) + (gamepad2.left_bumper ? 1 : 0), 1.0)) * 2 - 1);

        // Reset field-relative coordinate system when D-pad UP is pressed
        if (gamepad1.dpad_up) {
            angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            yawZero = 0.0 - angles.firstAngle;
        }

        // Drive controls
        if (!gamepad1.right_bumper) {
            // Field-relative mode (FIXED VERSION)
            driveFieldRelativeFixed(jsrc(gamepad1.left_stick_y), jsrc(gamepad1.left_stick_x), jsrc(gamepad1.right_stick_x));
        } else {
            // Robot-relative mode
            telemetry.addLine("ROBOT-RELATIVE MODE");
            drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        if (gamepad1.x && !lastGamepadX) {
            shooterToggle = !shooterToggle;
        }

        lastGamepadX = gamepad1.x;

        aToggler.update(gamepad1.a);

        double shooterPowerCoef = webInterface.getParameter("shooter_power");
        double upperWheelPower = webInterface.getParameter("feeder2_power");
        double restPowerLevel = 1.9;

        shooter.setPower(shooterToggle ? -shooterPowerCoef : (-invert_all_emergency * (((gamepad1.left_trigger * shooterPowerCoef - gamepad1.right_trigger * shooterPowerCoef) + (gamepad2.left_trigger * shooterPowerCoef - gamepad2.right_trigger * shooterPowerCoef))/2.0)));

        intake.setPower(avg(gamepad2.left_stick_y * restPowerLevel,   invert_all_emergency *   boolToDoubleBecauseItWontCast(aToggler.currentState || gamepad1.b || gamepad1.y) * restPowerLevel));
        feeder.setPower(avg(gamepad2.right_stick_y * restPowerLevel,  invert_all_emergency *  boolToDoubleBecauseItWontCast(gamepad1.b || gamepad1.y) * restPowerLevel));
        feeder2.setPower(avg(gamepad2.right_stick_x * upperWheelPower, invert_all_emergency * -boolToDoubleBecauseItWontCast(gamepad1.y) * upperWheelPower));

        if (gamepad2.aWasPressed()) {
            webInterface.setParameter("shooter_power", 0.55);
        }

        if (gamepad2.bWasPressed()) {
            webInterface.setParameter("shooter_power", webInterface.getParameter("shooter_power") + 0.05);
        }

        if (gamepad2.xWasPressed()) {
            webInterface.setParameter("shooter_power", webInterface.getParameter("shooter_power") - 0.05);
        }

        telemetry.addData("shooter_power", webInterface.getParameter("shooter_power"));

        telemetry.update();
    }

    // FIXED FIELD-RELATIVE DRIVE FUNCTION
    // The only change from original: minus changed to plus in theta calculation
    private void driveFieldRelativeFixed(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Get current robot orientation
        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Calculate yaw with normalization
        double yaw_corrected = ((((0.0 - angles.firstAngle) - this.yawZero) + 180.0)) % 360.0 - 180.0;

        // FIXED: Changed minus to plus here!
        theta = AngleUnit.normalizeRadians(theta + (yaw_corrected) * (Math.PI/180.0));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    private double jsrc(double power) {
        double a = 3.33;
        double b = 0.35;

        return (Math.signum(power)*Math.pow(Math.abs(power), a) + (power * b)) / (1 + b);
    }

    public void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
}