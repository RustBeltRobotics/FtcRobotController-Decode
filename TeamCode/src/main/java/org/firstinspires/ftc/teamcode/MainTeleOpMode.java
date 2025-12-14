package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.concurrent.TimeUnit;

@Disabled
@TeleOp
public class MainTeleOpMode extends LinearOpMode {
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    private DcMotor shootMotor;
    private DcMotor intakeMotor1;
    private DcMotor intakeMotor2;
    static final double POWER_FOR_DRIVER_SHOOT = 1;
    static final double POWER_FOR_DRIVER_INTAKE = 1;
    @Override
    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        shootMotor = hardwareMap.get(DcMotor.class, "shoot");
        intakeMotor1 = hardwareMap.get(DcMotor.class, "im1");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "im2");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IMU gyro = hardwareMap.get(IMU.class, "gyro");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        gyro.initialize(parameters);
        reset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        double denom = 0;
        double tgtPowerY = 0;
        double tgtPowerX = 0;
        double tgtPowerRX = 0;
        double botHeading = gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        while (opModeIsActive()) {
            botHeading = gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            telemetry.addData("botHeading", botHeading);
            tgtPowerX = this.gamepad1.left_stick_x;
            tgtPowerY = -this.gamepad1.left_stick_y;
            tgtPowerRX = jeySteckRespencse(-this.gamepad1.right_stick_x) * 0.64;

// CAREFUL TESTING THIS!! this is to give us good control at slow speeds, while still being able to move fast.
            tgtPowerX = jeySteckRespencse(tgtPowerX);
            tgtPowerY = jeySteckRespencse(tgtPowerY);

            // enable field-centric adjustments when left bumper pushed
            // TODO change this to a toggle on one of the sticks
            if (this.gamepad1.left_bumper == true) {
                tgtPowerX = tgtPowerX * Math.cos(-botHeading) - tgtPowerY * Math.sin(-botHeading);
                tgtPowerY = tgtPowerY * Math.sin(-botHeading) + tgtPowerY * Math.cos(-botHeading);
                tgtPowerX = tgtPowerX * 1.1;
                telemetry.addData("Field centric", "ON");
                telemetry.addData("botheading", botHeading);
            }
            if (this.gamepad2.right_bumper) {
                reset();
            }
            if (this.gamepad1.left_trigger > 0.1) {
                telemetry.addData("Intake value: ", this.gamepad1.left_trigger);
                runIntake(POWER_FOR_DRIVER_INTAKE);
            }
            if (this.gamepad1.right_trigger > 0.1) {
                telemetry.addData("Shooter value: ", this.gamepad1.right_trigger);
                runIntake(POWER_FOR_DRIVER_SHOOT);
            }

            denom = Math.max(Math.abs(tgtPowerY) + Math.abs(tgtPowerX) + Math.abs(tgtPowerRX), 1);

            motor1.setPower(-(tgtPowerY + tgtPowerX + tgtPowerRX) / -denom );
//            telemetry.addData("Target Power",(tgtPowerY + tgtPowerX + tgtPowerRX) / -denom );
//            telemetry.addData("Motor1 Power", motor1.getPower());
////            telemetry.addData("Motor1 RunMode", motor1.getMode());
            telemetry.addData("motor 1 position", motor1.getCurrentPosition());
//
            motor2.setPower(-(tgtPowerY + -tgtPowerX + -tgtPowerRX) / denom );
//            telemetry.addData("Target Power", (tgtPowerY + -tgtPowerX + -tgtPowerRX) / denom);
//            telemetry.addData("Motor2 Power", motor2.getPower());
////            telemetry.addData("Motor2 RunMode", motor2.getMode());
            telemetry.addData("motor 2 position", motor2.getCurrentPosition());
//
            motor3.setPower(-(tgtPowerY + -tgtPowerX + tgtPowerRX) / -denom );
//            telemetry.addData("Target Power", (tgtPowerY + -tgtPowerX + tgtPowerRX) / -denom);
//            telemetry.addData("Motor3 Power", motor3.getPower());
////            telemetry.addData("Motor3 RunMode", motor3.getMode());
            telemetry.addData("motor 3 position", motor3.getCurrentPosition());
//
            motor4.setPower(-(tgtPowerY + tgtPowerX + -tgtPowerRX) / denom );
//            telemetry.addData("Target Power", (tgtPowerY + tgtPowerX + -tgtPowerRX) / denom);
//            telemetry.addData("Motor4 Power", motor4.getPower());
            telemetry.addData("motor 4 position", motor4.getCurrentPosition());
////            telemetry.addData("Motor4 RunMode", motor4.getMode());

            // OPERATOR CONTROLLER
            if (gamepad2.x && gamepad2.a) {
                // climbHold();
            }

            if (gamepad2.y) {
                // upperScorePosition();
            }

            if (gamepad2.b) {
                // setArmToRunMode();
            }


            telemetry.addData("Status", "Running v6");
            telemetry.update();
        }
        // shutdown code

    }
    double jeySteckRespencse(double power) {
        double a = 3.7;
        double b = 0.43;

        return (Math.signum(power)*Math.pow(Math.abs(power), a) + (power * b)) / (1 + b);
    }
    private void reset() {
        shootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void runShooter(double shooterTgtPower) {
        shootMotor.setPower(shooterTgtPower);
        telemetry.addData("Shooter Tgt Power", shooterTgtPower);
    }


    private void runIntake(double intakeTgtPower){
        intakeMotor1.setPower(intakeTgtPower);
        intakeMotor2.setPower(intakeTgtPower);
        telemetry.addData("Intake Tgt Power", intakeTgtPower);
    }
}
