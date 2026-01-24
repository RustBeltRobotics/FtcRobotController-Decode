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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    DriveController driveController;
    double yawZero;
    double lastFL = 0, lastFR = 0, lastBL = 0, lastBR = 0;

    // Max power change per loop (tune this)
    static final double MAX_DELTA = 0.06;  // start here (SKEW)
    /*
    0.02	Ultra smooth, very safe
    0.04	Recommended start
    0.06	Aggressive but controlled
    0.08	May slip on strafes
    0.10+	Basically no ramp
    */
    // This declares the IMU needed to get the current direction the robot is facing
//    IMU imu;
    private double slew(double target, double current, double maxDelta) {
        double delta = target - current;
        if (delta > maxDelta) return current + maxDelta;
        if (delta < -maxDelta) return current - maxDelta;
        return target;
    }
    private double smartExpo(double x) {
        double softExp = 3.0;   // center precision
        double hardExp = 3;   // edge authority
        double blendStart = 0.25;
        double blendEnd   = 0.85;

        double ax = Math.abs(x);

        // Blend factor (0 → 1)
        double t = (ax - blendStart) / (blendEnd - blendStart);
        t = Math.max(0.0, Math.min(1.0, t));

        // Smoothstep for slope continuity
        t = t * t * (3 - 2 * t);

        double exp = softExp * (1 - t) + hardExp * t;

        return Math.signum(x) * Math.pow(ax, exp);

        // More finesse near center:
        // softExp ↑   (3.2 – 3.6)

        // more punch at full stick
        // hardExp ↓   (1.3 – 1.5)

        // Earlier Agression
        // blendStart ↓

        //Later Aggression
        //blendEnd ↑
    }


    private double jsrcOptimized(double x) {
        double softExp = 3.2;   // center precision (like your 3.33)
        double hardExp = 1.6;   // top-end control (prevents slip)

        double blendStart = 0.20;
        double blendEnd   = 0.85;

        double ax = Math.abs(x);

        // Smooth blend factor (smoothstep)
        double t = (ax - blendStart) / (blendEnd - blendStart);
        t = Math.max(0.0, Math.min(1.0, t));
        t = t * t * (3 - 2 * t);

        double exp = softExp * (1.0 - t) + hardExp * t;
        return Math.signum(x) * Math.pow(ax, exp);
    }


    @Override
    public void runOpMode() {
        Rev9AxisImu.Parameters parameters = new Rev9AxisImu.Parameters(new Rev9AxisImuOrientationOnRobot(Rev9AxisImuOrientationOnRobot.LogoFacingDirection.UP, Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.BACKWARD));


        imu = hardwareMap.get(Rev9AxisImu.class, "external_imu");
        imu.initialize(parameters);

        webTelemetryStreamer = new WebTelemetryStreamer(8886);
        Thread webTelemetryStreamerThread = new Thread(webTelemetryStreamer);
        webTelemetryStreamerThread.start(); // start server

        webInterface = new WebInterface(8885);
        webInterface.addParameter("shooter_power", 0.55);
        webInterface.addParameter("feeder2_power", 0.4);

        webInterface.addParameter("Kp_drive", 0.01);
        webInterface.addParameter("Ki_drive", 0.00);
        webInterface.addParameter("Kd_drive", 0.00);

        webInterface.addParameter("dbsizec", 0.3);
        webInterface.addParameter("dbdepthc", 0.3);

        Thread webInterfaceThread = new Thread(webInterface);
        webInterfaceThread.start(); // start server

//        SoundPlayer soundPlayer = new SoundPlayer(1, 4096);
//        soundPlayer.play()
//        SoundPlayer.getInstance().startPlaying();

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

        driveController = new DriveController(imu, frontRightDrive, frontLeftDrive, backLeftDrive, backRightDrive, new PIDController(0.0, 0.0, 0.0));
        driveController.init();

        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        driveController.yawZero = 0.0 - angles.firstAngle;
//        yawZero = 0.0 - angles.firstAngle;

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

        driveController.drivingPID.setCoefs(
                webInterface.getParameter("Kp_drive"),
                webInterface.getParameter("Ki_drive"),
                webInterface.getParameter("Kd_drive")
        );
        driveController.drivingPID.deadbandSizeCoef = webInterface.getParameter("dbsizec");
        driveController.drivingPID.deadbandDepthCoef = webInterface.getParameter("dbdepthc");

//        System.out.println(webInterface.getParameter("Kp_drive"));
//        System.out.println(driveController.drivingPID.Kp);
//        System.out.println(driveController.drivingPID.Ki);
//        System.out.println(driveController.drivingPID.Kd);

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

            webTelemetryStreamer.sendData("PIDTarget", (180/3.14159) * driveController.drivingPID.target);
            webTelemetryStreamer.sendData("PIDOutput", driveController.drivingPID.output);

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

            // get voltage sensor (there are two one is on the expansion hub and this is not ideal as it does not specify which one to use)
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
//                webTelemetryStreamer.sendData("battery_voltage", sensor.getVoltage() * 100); // multiply by 100 so it is in a similar range to other values
                break;
            }
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


        /**** origional configuration of the joystick ******/
        ////////////double y  = jsrc(gamepad1.left_stick_y);     // forward/back
        ////////////double x  = jsrc(gamepad1.left_stick_x);     // strafe
        //double rx = gamepad1.right_stick_x * 0.9;    // rotation ("LINEAR", NO CURVE)
        /************** other options for configurations of joysticks ************/
        //double rx = gamepad1.right_stick_x * 1.0;   //more snap
        //double rx = gamepad1.right_stick_x * 0.7;   // smoother turning
        //double rx = Math.signum(gamepad1.right_stick_x) * Math.pow(Math.abs(gamepad1.right_stick_x), 1.5); // little curve on rotation
        /*********** ercommended control configuration *******/
        //double y = expoMix(gamepad1.left_stick_y, 1.8, 0.25);
        //double x  = expoMix(gamepad1.left_stick_x, 1.8, 0.25);
        /////////double rx  = expoMix(gamepad1.right_stick_x, 2.8, 0.15);
        /*************** Tucker Edit V3 for controller configuration***********/
        //double y = expoMix(gamepad1.left_stick_y, 1.7, 0.30);
        //double x = expoMix(gamepad1.left_stick_x, 1.7, 0.30);
        //double rx = expoMix(gamepad1.right_stick_x, 2.2, 0.20);
        /*************** Tucker Edit V4 for controller configuration***********/
        double y  = smartExpo(-gamepad1.left_stick_y);
        double x  = smartExpo(-gamepad1.left_stick_x);
        double rx = smartExpo(gamepad1.right_stick_x) * 0.9;
        /*************** Tucker Edit V for controller configuration***********/
        //double y  = jsrcOptimized(gamepad1.left_stick_y);
        //double x  = jsrcOptimized(gamepad1.left_stick_x);
        //double rx = jsrcOptimized(gamepad1.right_stick_x) * 0.9;





        if (!gamepad1.right_bumper) {
            driveFieldRelative(y, x, rx);   // ← YOUR polar math
        } else {
            telemetry.addLine("RIGHT BUMPER");
            drive(y, x, rx);               // robot-relative
        }


        if (gamepad1.x && !lastGamepadX) {
            shooterToggle = !shooterToggle;
        }
        if (gamepad1.dpad_up) {
            driveController.yawZero = 0.0 - angles.firstAngle;
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

//        gamepad2.resetEdgeDetection();
        telemetry.update();
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    private double expoMix(double input, double exponent, double linearWeight) {
        return (1.0 - linearWeight) * Math.signum(input) * Math.pow(Math.abs(input), exponent)
                + linearWeight * input;
    }
    private double jsrc(double power) {
//        double a = 3.7;
//        double b = 0.43;
        //double a = 3.33;
        //double b = 0.35;
        double a = 2.5;
        double b = 0.1;

        return (Math.signum(power)*Math.pow(Math.abs(power), a) + (power * b)) / (1 + b);
    }

    // This routine drives the robot field relative
    /***
     private void driveFieldRelative(double forward, double right, double rotate) {

        // Get robot heading in radians
        double heading = Math.toRadians(angles.firstAngle) - yawZero;

        // Rotate joystick vector by robot heading
        double rotForward =  forward * Math.cos(heading) + right * Math.sin(heading);
        double rotRight   = -forward * Math.sin(heading) + right * Math.cos(heading);

        drive(rotForward, rotRight, rotate);
    }***/
    /***** working drive field relative code
    private void driveFieldRelative(double forward, double right, double rotate) {

        double heading = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);

        double cosA = Math.cos(heading);
        double sinA = Math.sin(heading);

        double robotForward = forward * cosA + right * sinA;
        double robotRight   = -forward * sinA + right * cosA;

        drive(robotForward, robotRight, rotate);
    }***/
    private void driveFieldRelative(double forward, double right, double rotate) {

        // --- JOYSTICK VECTOR ---
        double mag = Math.hypot(forward, right);
        if (mag < 0.05) mag = 0.0;
        mag = Math.min(mag, 1.0);   // 🔥 THIS FIXES DIAGONALS

        double angle = Math.atan2(forward, right);

        // --- APPLY EXPO TO MAGNITUDE ONLY ---
        double shapedMag = smartExpo(mag);

        // Rebuild vector
        double f = shapedMag * Math.sin(angle);
        double r = shapedMag * Math.cos(angle);

        // --- FIELD RELATIVE ROTATION ---
        double heading = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);

        double cosA = Math.cos(heading);
        double sinA = Math.sin(heading);

        double robotForward =  -f * cosA + r * sinA;
        double robotRight   = -f * sinA + r * cosA;

        // Rotation expo is OK
        rotate = smartExpo(rotate) * 0.85;

        drive(robotForward, robotRight, rotate);
    }





    // Thanks to FTC16072 for sharing this code!!
    /********************Start origional drive code that works **************/

    /**************
    public void drive(double forward, double right, double rotate) {

        // --- JOYSTICK CIRCLE PRESERVATION ---
        double transMag = Math.hypot(forward, right);
        if (transMag > 1.0) {
            forward /= transMag;
            right   /= transMag;
            transMag = 1.0;
        }

        // --- ROTATION RESERVATION (KEY IMPROVEMENT) ---
        double rotMag = Math.abs(rotate);

        // Smooth nonlinear reservation curve
        double transScale = 1.0 - 0.6 * rotMag * rotMag;
        forward *= transScale;
        right   *= transScale;

        // --- MECANUM MIX ---
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;

        // --- FINAL SOFT SATURATION ---
        double max = Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))
        );

        if (max > 1.0) {
            double scale = 1.0 / max;
            fl *= scale;
            fr *= scale;
            bl *= scale;
            br *= scale;
        }

        // --- SLEW RATE LIMITING ---
        fl = slew(fl, lastFL, MAX_DELTA);
        fr = slew(fr, lastFR, MAX_DELTA);
        bl = slew(bl, lastBL, MAX_DELTA);
        br = slew(br, lastBR, MAX_DELTA);

        lastFL = fl;
        lastFR = fr;
        lastBL = bl;
        lastBR = br;

        // --- OUTPUT ---
        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
    }
    *****************/

//    public void drive(double forward, double right, double rotate) {
//
//        // Raw wheel powers
//        double fl = forward + right + rotate;
//        double fr = forward - right - rotate;
//        double br = forward + right - rotate;
//        double bl = forward - right + rotate;
//
//        // Normalize ONLY if needed
//        double maxMag = Math.max(
//                Math.max(Math.abs(fl), Math.abs(fr)),
//                Math.max(Math.abs(bl), Math.abs(br))
//        );
//
//        if (maxMag > 1.0) {
//            fl /= maxMag;
//            fr /= maxMag;
//            br /= maxMag;
//            bl /= maxMag;
//        }
//
//        // Optional global speed limit
//        double maxSpeed = 1.0;
//        fl *= maxSpeed;
//        fr *= maxSpeed;
//        br *= maxSpeed;
//        bl *= maxSpeed;
//
//        // Slew rate limiting (FINAL step)
//        double flOut = slew(fl, lastFL, MAX_DELTA);
//        double frOut = slew(fr, lastFR, MAX_DELTA);
//        double blOut = slew(bl, lastBL, MAX_DELTA);
//        double brOut = slew(br, lastBR, MAX_DELTA);
//
//        // Save for next loop
//        lastFL = flOut;
//        lastFR = frOut;
//        lastBL = blOut;
//        lastBR = brOut;
//
//        // Send to motors
//        frontLeftDrive.setPower(flOut);
//        frontRightDrive.setPower(frOut);
//        backLeftDrive.setPower(blOut);
//        backRightDrive.setPower(brOut);
//    }
    /******************************************************************/



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

