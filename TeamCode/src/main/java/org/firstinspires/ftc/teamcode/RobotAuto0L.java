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
@Autonomous(name = "RobotAuto0L", group = "Robot")
public class RobotAuto0L extends LinearOpMode {
    // This declares the four motors needed

    int state = 0; // set to 3 to start driving immediately
    double stateStartTime = 0;

    boolean shooterToggle = false;
    boolean lastGamepadX = false;

    BasicAutoClass autoThing = new BasicAutoClass();

    // This declares the IMU needed to get the current direction the robot is facing
//    IMU imu;

    @Override
    public void runOpMode() {
        autoThing.onRunOpMode(this, (Integer state, Double stateStartTime) -> {
            // blue shoot and aquire
            if (state == 0) {
                // spin up shooter
                telemetry.addLine("Shooter spin-up...");
                autoThing.shooter.setPower(-0.55);

                autoThing.driveController.driveFieldRelative(0.0, 0.0, 0);

                if (getRuntime() - stateStartTime > 0.5) {
                    state++;
                    stateStartTime = getRuntime();
                }
            } else if (state == 1) {
                autoThing.driveController.driveFieldRelative(0.25, 0.0, 0);

                if (getRuntime() - stateStartTime > 0.2) {
                    state++;
                    stateStartTime = getRuntime();
                }
            } else if (state == 2) {
//            driveController.driveFieldRelativeAuto(0.0, 0.0, 0);
//            driveController.stop();

                if (getRuntime() - stateStartTime > 2.3) {
                    state++;
                    stateStartTime = getRuntime();
                }
            } else if (state == 3) {
                // shoot balls
                telemetry.addLine("Shooting balls...");
                autoThing.intake.setPower(1.9);
                autoThing.feeder.setPower(1.9);
                autoThing.feeder2.setPower(-0.4);

                if (getRuntime() - stateStartTime > 2.0) {
//                shooter.setPower(0); // don't disable shooter because...
                    autoThing.intake.setPower(0);
                    autoThing.feeder.setPower(0);
                    autoThing.feeder2.setPower(0);

                    state++;
//                state = -1; // end
                    stateStartTime = getRuntime();
                }
            } else if (state == 4) {
                telemetry.addLine("Drive 1...");
                autoThing.driveController.driveFieldRelative(0.125, 0.0, 0);
                if (getRuntime() - stateStartTime > 1.4) {
                    state++;
//                state = -1; // end
                    stateStartTime = getRuntime();
                }
            } else if (state == 5) {
                telemetry.addLine("Drive 2...");
                autoThing.driveController.driveFieldRelative(0.0, -0.125, 0);

                autoThing.intake.setPower(1.9);
                autoThing.feeder.setPower(1.9);

                if (getRuntime() - stateStartTime > 1.25) {
                    state++;
//                state = -1; // end
                    stateStartTime = getRuntime();
                }
            } else if (state == 6) {

                if (getRuntime() - stateStartTime > 1.0) {
                    state++;
//                state = -1; // end
                    stateStartTime = getRuntime();
                }
            }

            return new AutoStateThing(state, stateStartTime);
        });

    }


}
