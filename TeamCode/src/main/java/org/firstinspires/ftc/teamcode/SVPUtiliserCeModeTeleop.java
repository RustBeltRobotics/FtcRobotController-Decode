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
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.IOException;
import java.util.Locale;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@TeleOp(name = "Robot: Field Relative Mecanum Drive", group = "Robot")
public class SVPUtiliserCeModeTeleop extends LinearOpMode {
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

    VoltageSensor voltageSensor;

    boolean shooterToggle = false;
    boolean lastGamepadX = false;

    Toggler aToggler = new Toggler();

    DriveController driveController;

    double yawZero;


    // This declares the IMU needed to get the current direction the robot is facing
//    IMU imu;

    @Override
    public void runOpMode() {

        Rev9AxisImu.Parameters parameters = new Rev9AxisImu.Parameters(new Rev9AxisImuOrientationOnRobot(Rev9AxisImuOrientationOnRobot.LogoFacingDirection.UP, Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.BACKWARD));


        imu = hardwareMap.get(Rev9AxisImu.class, "external_imu");
        imu.initialize(parameters);

        webTelemetryStreamer = new WebTelemetryStreamer(8886);
        Thread webTelemetryStreamerThread = new Thread(webTelemetryStreamer);
        webTelemetryStreamerThread.start(); // start server

        webInterface = new WebInterface(8885);
        webInterface.addParameter("shooter_power", 0.85);
        webInterface.addParameter("feeder2_power", 0.30);
        Thread webInterfaceThread = new Thread(webInterface);
        webInterfaceThread.start(); // start server

//        SoundPlayer soundPlayer = new SoundPlayer(1, 4096);
//        soundPlayer.play()
//        SoundPlayer.getInstance().startPlaying();

        voltageSensor = hardwareMap.get(VoltageSensor.class, "voltage_sensor");


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

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
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



//        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

//        RevHubOrientationOnRobot orientationOnRobot = new
//                RevHubOrientationOnRobot(logoDirection, usbDirection);
//        imu.initialize(new IMU.Parameters(orientationOnRobot));


        waitForStart();

        //        driveController = new DriveController(imu, frontRightDrive, frontLeftDrive, backLeftDrive, backRightDrive, new PIDController(1.0, 0.1, 0.3));
//        driveController.init();

        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        driveController.yawZero = 0.0 - angles.firstAngle;
        yawZero = 0.0 - angles.firstAngle;

        int loopcounter = 0;
        while (opModeIsActive()) {
            loop2(loopcounter++);
        }

//        try {
//            webInterface.stop();
//        } catch (IOException e) {}

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
        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");

        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("connection", imu.getConnectionInfo());

        // wrong labels
        telemetry.addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.addData("roll", formatAngle(angles.angleUnit, angles.secondAngle));
        telemetry.addData("pitch", formatAngle(angles.angleUnit, angles.thirdAngle));

        // send slower to stop network getting backed up (maybe it would help if I don't flush data inside of sendData - oh wait I'm already doing that)
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

            webTelemetryStreamer.sendData("battery_voltage", voltageSensor.getVoltage());
        }

        // TODO: make opmode just to test every motors' current draw individually under no load

        double invert_all_emergency = ((Math.min((gamepad1.left_bumper ? 1 : 0) + (gamepad2.left_bumper ? 1 : 0), 1.0)) * 2 - 1);

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
//        if (gamepad1.a) {
//            imu.resetYaw();
//        }
        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)

//        drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // 90° rotation
//        drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (!gamepad1.right_bumper) {
//           driveController.driveFieldRelative(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            driveFieldRelative(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            telemetry.addLine("RIGHT BUMPER");
            drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

//        if (gamepad1.left_bumper) {
//            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//        } else {
//            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//        }

        if (gamepad1.x && !lastGamepadX) {
            shooterToggle = !shooterToggle;
        }

        if (gamepad1.dpad_up) {
            yawZero = 0.0 - angles.firstAngle;
        }

        lastGamepadX = gamepad1.x;


        aToggler.update(gamepad1.a);


        double shooterPowerCoef = webInterface.getParameter("shooter_power"); // 0.80;
        double upperWheelPower = webInterface.getParameter("feeder2_power");  // 0.3
        double restPowerLevel = 1.9; // 1.9


        shooter.setPower(shooterToggle ? -shooterPowerCoef : (-invert_all_emergency * (((gamepad1.left_trigger * shooterPowerCoef - gamepad1.right_trigger * shooterPowerCoef) + (gamepad2.left_trigger * shooterPowerCoef - gamepad2.right_trigger * shooterPowerCoef))/2.0)));

        intake.setPower(avg(gamepad2.left_stick_y * restPowerLevel,   invert_all_emergency *   boolToDoubleBecauseItWontCast(aToggler.currentState || gamepad1.b || gamepad1.y) * restPowerLevel));
        feeder.setPower(avg(gamepad2.right_stick_y * restPowerLevel,  invert_all_emergency *  boolToDoubleBecauseItWontCast(gamepad1.b || gamepad1.y) * restPowerLevel));
        feeder2.setPower(avg(gamepad2.right_stick_x * upperWheelPower, invert_all_emergency * -boolToDoubleBecauseItWontCast(gamepad1.y) * upperWheelPower));
        telemetry.update();
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        // reverse angle because the robot magically got mixed up and the angle was inverted

        double yaw_corrected = ((((0.0 - angles.firstAngle) - this.yawZero) + 180.0)) % 360.0 - 180.0;

        theta = AngleUnit.normalizeRadians(theta - (yaw_corrected) * (3.141592653589/180));//AngleUnit.normalizeRadians(theta -
                //imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

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
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
}
