package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.IOException;
import java.util.Locale;
import java.util.function.BiFunction;

public class BasicAutoClass {

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

    WebInterface webInterface;
    WebTelemetryStreamer webTelemetryStreamer;

    int state = 0; // set to 3 to start driving immediately
    double stateStartTime = 0;

    LinearOpMode opMode;

    BiFunction<Integer, Double, Void> opModeLoopFunction;

    void onRunOpMode(LinearOpMode opMode, BiFunction<Integer, Double, Void> opModeLoopFunction) {

        this.opModeLoopFunction = opModeLoopFunction;

        this.opMode = opMode;
        Rev9AxisImu.Parameters parameters = new Rev9AxisImu.Parameters(new Rev9AxisImuOrientationOnRobot(Rev9AxisImuOrientationOnRobot.LogoFacingDirection.UP, Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.BACKWARD));


        // BNO055IMUNew
        imu = opMode.hardwareMap.get(Rev9AxisImu.class, "external_imu");
        imu.initialize(parameters);

//        webInterface = new WebInterface(8000 + (int)(Math.random() * 800)); // terrible workaround to not releasing port properly
        webInterface = new WebInterface(8885);
        webInterface.addParameter("Kp_drive", 0.1);
        webInterface.addParameter("Ki_drive", 0.02);
        webInterface.addParameter("Kd_drive", 0.05);

        webInterface.addParameter("dbsizec", 0.3);
        webInterface.addParameter("dbdepthc", 0.3);

        Thread webInterfaceThread = new Thread(webInterface);
        webInterfaceThread.start(); // start server

        webTelemetryStreamer = new WebTelemetryStreamer(8886);
        Thread webTelemetryStreamerThread = new Thread(webTelemetryStreamer);
        webTelemetryStreamerThread.start(); // start server


        frontLeftDrive = opMode.hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = opMode.hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = opMode.hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = opMode.hardwareMap.get(DcMotor.class, "back_right_drive");

        shooter = opMode.hardwareMap.get(DcMotor.class, "shooter");


        intake = opMode.hardwareMap.get(DcMotor.class, "intake");
        feeder = opMode.hardwareMap.get(DcMotor.class, "feeder");
        feeder2 = opMode.hardwareMap.get(DcMotor.class, "feeder2");

        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        feeder.setDirection(DcMotor.Direction.REVERSE);
        feeder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        feeder2.setDirection(DcMotor.Direction.REVERSE);
        feeder2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        driveController = new DriveController(imu, frontRightDrive, frontLeftDrive, backLeftDrive, backRightDrive, new PIDController(0.0, 0.0, 0.0));
        driveController.init();

        driveController.drivingPID.deadbandSizeCoef = webInterface.getParameter("dbsizec");
        driveController.drivingPID.deadbandDepthCoef = webInterface.getParameter("dbdepthc");

        updateDrivingPIDCoefs();


        opMode.waitForStart();

        stateStartTime = opMode.getRuntime();

//        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

//        System.out.print("yaw before resetYaw: ");
//        System.out.println(angles.firstAngle);

        imu.resetYaw(); // aha this is the culprit!! this doesn't work!!! argh

        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

//        System.out.print("yaw after resetYaw: ");
//        System.out.println(angles.firstAngle);

        driveController.yawZero = 0.0 - angles.firstAngle;

        int loopcounter = 0;
        while (opMode.opModeIsActive()) {
            loop2(loopcounter++);
        }

        try {
            webInterface.stop();
        } catch (IOException e) {}

        try {
            webTelemetryStreamer.stop();
        } catch (IOException e) {}
    }


    private void updateDrivingPIDCoefs() {
        driveController.drivingPID.setCoefs(
                webInterface.getParameter("Kp_drive"),
                webInterface.getParameter("Ki_drive"),
                webInterface.getParameter("Kd_drive")
        );
    }

    private double avg(double a, double b) {
        return (a + b)/2;
    }

    private double boolToDoubleBecauseItWontCast(boolean input) {
        return input ? 1.0 : 0.0;
    }

    public void loop2(int loopcounter) {
        opMode.telemetry.addLine("Press A to reset Yaw");
        opMode.telemetry.addLine("Hold left bumper to drive in robot relative");
        opMode.telemetry.addLine("The left joystick sets the robot direction");
        opMode.telemetry.addLine("Moving the right joystick left and right turns the robot");

        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        opMode.telemetry.addData("connection", imu.getConnectionInfo());

        opMode.telemetry.addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));
        opMode.telemetry.addData("roll", formatAngle(angles.angleUnit, angles.secondAngle));
        opMode.telemetry.addData("pitch", formatAngle(angles.angleUnit, angles.thirdAngle));



        if (loopcounter % 3 == 0) {
            webTelemetryStreamer.sendData("heading", angles.firstAngle);
            webTelemetryStreamer.sendData("roll", angles.secondAngle);
            webTelemetryStreamer.sendData("pitch", angles.thirdAngle);

            webTelemetryStreamer.sendData("PIDTarget", (180/3.14159) * driveController.drivingPID.target);
            webTelemetryStreamer.sendData("PIDOutput", driveController.drivingPID.output);

            // 0.01 scale just to make it fit in with the other units (todo: make multiple separate graphs each with one unit)
            webTelemetryStreamer.sendData("current_frontLeftDrive", 0.01 * ((DcMotorEx) frontLeftDrive).getCurrent(CurrentUnit.MILLIAMPS));
            webTelemetryStreamer.sendData("current_frontRightDrive", 0.01 * ((DcMotorEx) frontRightDrive).getCurrent(CurrentUnit.MILLIAMPS));
            webTelemetryStreamer.sendData("current_backLeftDrive", 0.01 * ((DcMotorEx) backLeftDrive).getCurrent(CurrentUnit.MILLIAMPS));
            webTelemetryStreamer.sendData("current_backRightDrive", 0.01 * ((DcMotorEx) backRightDrive).getCurrent(CurrentUnit.MILLIAMPS));
        }



        // This does not work!
//        telemetry.addData("frontRightDrive Current", frontRightDrive.getCurrent(CurrentUnit.MILLIAMPS));

        // This works!
//        telemetry.addData("frontRightDrive Current", ((DcMotorControllerEx) frontRightDrive.getController()).getMotorCurrent(frontRightDrive.getPortNumber(), CurrentUnit.MILLIAMPS));

        // This works!
        opMode.telemetry.addData("frontLeftDrive Current", ((DcMotorEx) frontLeftDrive).getCurrent(CurrentUnit.MILLIAMPS));
        opMode.telemetry.addData("frontRightDrive Current", ((DcMotorEx) frontRightDrive).getCurrent(CurrentUnit.MILLIAMPS));
        opMode.telemetry.addData("backLeftDrive Current", ((DcMotorEx) backLeftDrive).getCurrent(CurrentUnit.MILLIAMPS));
        opMode.telemetry.addData("backRightDrive Current", ((DcMotorEx) backRightDrive).getCurrent(CurrentUnit.MILLIAMPS));

        // update PID coefs from webInterface
        updateDrivingPIDCoefs();

        this.opModeLoopFunction.apply(state, stateStartTime);


        opMode.telemetry.update();
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}

