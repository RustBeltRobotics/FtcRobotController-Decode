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

import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
@Autonomous(name = "Robot: Field Relative Mecanum Drive", group = "Robot")
public class RobotAuto0L extends LinearOpMode {
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

    DriveController driveController;

    int state = 3; // set to 3 to start driving immediately
    double stateStartTime = 0;

    boolean shooterToggle = false;
    boolean lastGamepadX = false;


    // This declares the IMU needed to get the current direction the robot is facing
//    IMU imu;

    @Override
    public void runOpMode() {

        Rev9AxisImu.Parameters parameters = new Rev9AxisImu.Parameters(new Rev9AxisImuOrientationOnRobot(Rev9AxisImuOrientationOnRobot.LogoFacingDirection.FORWARD, Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.DOWN));


        imu = hardwareMap.get(Rev9AxisImu.class, "external_imu");
        imu.initialize(parameters);


        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        shooter = hardwareMap.get(DcMotor.class, "shooter");


        intake = hardwareMap.get(DcMotor.class, "intake");
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        feeder2 = hardwareMap.get(DcMotor.class, "feeder2");

        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        feeder.setDirection(DcMotor.Direction.REVERSE);
        feeder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        feeder2.setDirection(DcMotor.Direction.REVERSE);
        feeder2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        driveController = new DriveController(imu, frontRightDrive, frontLeftDrive, backLeftDrive, backRightDrive);
        driveController.init();

        


        waitForStart();

        stateStartTime = getRuntime();
        imu.resetYaw();

        while (opModeIsActive()) {
            loop2();
        }
    }

    private double avg(double a, double b) {
        return (a + b)/2;
    }

    private double boolToDoubleBecauseItWontCast(boolean input) {
        return input ? 1.0 : 0.0;
    }

    public void loop2() {
        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");

        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("connection", imu.getConnectionInfo());

        telemetry.addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.addData("roll", formatAngle(angles.angleUnit, angles.secondAngle));
        telemetry.addData("pitch", formatAngle(angles.angleUnit, angles.thirdAngle));

        // This does not work!
//        telemetry.addData("frontRightDrive Current", frontRightDrive.getCurrent(CurrentUnit.MILLIAMPS));

        // This works!
//        telemetry.addData("frontRightDrive Current", ((DcMotorControllerEx) frontRightDrive.getController()).getMotorCurrent(frontRightDrive.getPortNumber(), CurrentUnit.MILLIAMPS));

        // This works!
        telemetry.addData("frontLeftDrive Current", ((DcMotorEx) frontLeftDrive).getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("frontRightDrive Current", ((DcMotorEx) frontRightDrive).getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("backLeftDrive Current", ((DcMotorEx) backLeftDrive).getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("backRightDrive Current", ((DcMotorEx) backRightDrive).getCurrent(CurrentUnit.MILLIAMPS));


        if (state == 0) {
            // spin up shooter
            telemetry.addLine("Shooter spin-up...");
            shooter.setPower(-1.9);

            if (getRuntime() - stateStartTime > 3.0) {
                state = 1;
                stateStartTime = getRuntime();
            }
        } else if (state == 1) {
            // shoot balls
            telemetry.addLine("Shooting balls...");
            intake.setPower(-1.9);
            feeder.setPower(1.9);
            feeder2.setPower(-1.9);

            if (getRuntime() - stateStartTime > 2.0) {
                shooter.setPower(0);
                intake.setPower(0);
                feeder.setPower(0);
                feeder2.setPower(0);

                state = 2;
                stateStartTime = getRuntime();
            }
        } else if (state == 2) {
            telemetry.addLine("Drive 1...");
            driveController.driveFieldRelative(0, -0.3, 0);
            if (getRuntime() - stateStartTime > 1.0) {
                state = 3;
                stateStartTime = getRuntime();
            }
        } else if (state == 3) {
            telemetry.addLine("Drive 2...");
            driveController.driveFieldRelative(-0.4, 0, 0);
            if (getRuntime() - stateStartTime > 3.0) {
                state = 4;
                stateStartTime = getRuntime();
            }
        } else if (state == 4) {
//            driveFieldRelative(0, 0, 0);

            driveController.stop();
        }

        telemetry.update();
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
