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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
@Autonomous(name = "!_BlueShoot9", group = "Robot")
public class AutoBlueShoot6New extends LinearOpMode {

    BasicAutoClass autoThing = new BasicAutoClass();

    @Override
    public void runOpMode() {
        autoThing.onRunOpMode(this, (Integer state, Double stateStartTime) -> {
            // blue shoot, aquire and shoot
            if (state == 0) {
                telemetry.addLine("Shooter spin-up...");
                autoThing.shooter.setPower(-0.55);
                autoThing.driveController.driveFieldRelative(0.0, 0.0, 0);
                if (getRuntime() - stateStartTime > 0.5) {state++;stateStartTime = getRuntime();
                }
            } else if (state == 1) {
                telemetry.addLine("Drive backwards off of goal position");
                autoThing.driveController.driveFieldRelative(0.25, 0.0, 0);
                if (getRuntime() - stateStartTime > 0.2) { state++; stateStartTime = getRuntime();
                }
            } else if (state == 2) {
                telemetry.addLine("Wait for shooter to spin up");
                if (getRuntime() - stateStartTime > 2.3) {state++;stateStartTime = getRuntime();
                }
            } else if (state == 3) {
                telemetry.addLine("Shooting balls...");
                autoThing.intake.setPower(1.9);
                autoThing.feeder.setPower(1.9);
                autoThing.feeder2.setPower(-0.4);
                if (getRuntime() - stateStartTime > 3.0) {
                    autoThing.intake.setPower(0);
                    autoThing.feeder.setPower(0);
                    autoThing.feeder2.setPower(0);
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 4) {
                telemetry.addLine("Drive Away from goal towards first row pickup spot");
                autoThing.driveController.driveFieldRelative(0.25, 0.0, 0); //(0.125, 0.0, 0)
                if (getRuntime() - stateStartTime > 0.9) {//(getRuntime() - stateStartTime > 2.0)
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 5) {
                telemetry.addLine("Drive Away from goal towards 1nd row pickup posiiton 2");
                autoThing.driveController.driveFieldRelative(0.22, -0.3, 0); //(-0.15, -0.5, 0) //tried to move back more and left less.
                if (getRuntime() - stateStartTime > 0.25) {//(getRuntime() - stateStartTime > 2.0)
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 6) {
                telemetry.addLine("Rotate to pickup balls");
                autoThing.driveController.driveFieldRelative(0.0, 0, Math.toRadians(60));
                if (getRuntime() - stateStartTime > 0.32) { //0.35
                    state++; stateStartTime = getRuntime();
                }
            } else if (state == 7) {
                telemetry.addLine("Strafe to pickup first row balls");
                autoThing.intake.setPower(1.9);
                autoThing.feeder.setPower(1.9);
                autoThing.driveController.driveFieldRelative(-0.125, -0.125, 0); //(-0.125, -0.125, 0)
                if (getRuntime() - stateStartTime > 1.95) { //(getRuntime() - stateStartTime > 2.0)
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 8) {
                telemetry.addLine("Strafe to back to first row start position");
                autoThing.intake.setPower(0);
                autoThing.feeder.setPower(0);
                autoThing.driveController.driveFieldRelative(0.125, 0.125, 0); //(0.125, 0.125, 0)
                if (getRuntime() - stateStartTime > 1.95) { //(getRuntime() - stateStartTime > 2.0)
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 9) {
                telemetry.addLine("Rotate back to face goal again.");
                autoThing.driveController.driveFieldRelative(0.0, 0, -Math.toRadians(60));
                if (getRuntime() - stateStartTime > 0.26) { //0.25)
                    state++;stateStartTime = getRuntime();
                }
            }
            else if (state == 10) {
                telemetry.addLine("Drive Away from first row pickup towards 2nd row pickup");
                autoThing.driveController.driveFieldRelative(-0.22, 0.3, 0); //(-0.15, -0.5, 0) //tried to move back more and left less.
                if (getRuntime() - stateStartTime > 0.25) {//(getRuntime() - stateStartTime > 2.0)
                    state++;stateStartTime = getRuntime();
                }
            }else if (state == 11) {
                telemetry.addLine("Drive towards goal");
                autoThing.driveController.driveFieldRelative(-0.25, 0.0, 0); //(0.125, 0.0, 0)
                if (getRuntime() - stateStartTime > 1.2) {//(getRuntime() - stateStartTime > 2.0)
                    state++; stateStartTime = getRuntime();
                }
            }else if (state == 12) {
                telemetry.addLine("drive away from goal slightly");
                autoThing.driveController.driveFieldRelative(0.25, 0.0, 0);
                if (getRuntime() - stateStartTime > 0.2) { state++;stateStartTime = getRuntime();
                }
            } else if (state == 13) {
                telemetry.addLine("Shooting balls...");
                autoThing.intake.setPower(1.9);
                autoThing.feeder.setPower(1.9);
                autoThing.feeder2.setPower(-0.4);
                if (getRuntime() - stateStartTime > 3.5) {
                    autoThing.intake.setPower(0);
                    autoThing.feeder.setPower(0);
                    autoThing.feeder2.setPower(0);
                    state++; stateStartTime = getRuntime();
                }
            }
            /***********************start of 2nd row pickup  *************************************************************/
            else if (state == 14) {
                telemetry.addLine("Drive Away from goal towards first row pickup spot");
                autoThing.driveController.driveFieldRelative(0.25, 0.0, 0); //(0.125, 0.0, 0)
                if (getRuntime() - stateStartTime > 0.9) {//(getRuntime() - stateStartTime > 2.0)
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 15) {
                telemetry.addLine("Drive Away from goal towards 2nd row pickup");
                autoThing.driveController.driveFieldRelative(0.22, -0.3, 0); //(-0.15, -0.5, 0) //tried to move back more and left less.
                if (getRuntime() - stateStartTime > 1.25) {//(getRuntime() - stateStartTime > 2.0)
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 16) {
                telemetry.addLine("Rotate to pickup balls");
                autoThing.driveController.driveFieldRelative(0.0, 0, Math.toRadians(60));
                if (getRuntime() - stateStartTime > 0.35) {state++;stateStartTime = getRuntime();
                }
            } else if (state == 17) {
                telemetry.addLine("Strafe to pickup first row balls");
                autoThing.driveController.driveFieldRelative(-0.125, -0.125, 0); //(-0.125, -0.125, 0)
                autoThing.intake.setPower(1.9);
                autoThing.feeder.setPower(1.9);
                if (getRuntime() - stateStartTime > 1.3) { //(getRuntime() - stateStartTime > 0.8)
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 18) {
                telemetry.addLine("Strafe back to initial pickup position first row balls");
                autoThing.driveController.driveFieldRelative(0.125, 0.125, 0); //(-0.125, -0.125, 0)
                autoThing.intake.setPower(0);
                autoThing.feeder.setPower(0);
                if (getRuntime() - stateStartTime > 0.8) { //(getRuntime() - stateStartTime > 2.0)
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 19) {
                telemetry.addLine("Rotate back to face goal again.");
                autoThing.driveController.driveFieldRelative(0.0, 0, -Math.toRadians(60));
                if (getRuntime() - stateStartTime > 0.35) {
                    state++; stateStartTime = getRuntime();
                }
            } else if (state == 20) {
                telemetry.addLine("Drive Away from first row pickup towards 2nd row pickup");
                autoThing.driveController.driveFieldRelative(-0.22, 0.33, 0); //(-0.2, -0.5, 0)
                if (getRuntime() - stateStartTime > 1.75) {//(getRuntime() - stateStartTime > 1.5)
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 21) {
                telemetry.addLine("Drive into goal and prepare to shoot");
                autoThing.driveController.driveFieldRelative(-0.25, 0.0, 0); //(-0.125, 0.0, 0)
                if (getRuntime() - stateStartTime > 0.9) { //(getRuntime() - stateStartTime > 2.5)
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 22) {
                telemetry.addLine("drive away from goal slightly");
                autoThing.driveController.driveFieldRelative(0.25, 0.0, 0);
                if (getRuntime() - stateStartTime > 0.2) { state++;stateStartTime = getRuntime();
                }
            }
            else if (state == 23) {
                telemetry.addLine("Shooting balls...");
                autoThing.intake.setPower(1.9);
                autoThing.feeder.setPower(1.9);
                autoThing.feeder2.setPower(-0.4);
                if (getRuntime() - stateStartTime > 4.5) {
                    autoThing.intake.setPower(0);
                    autoThing.feeder.setPower(0);
                    autoThing.feeder2.setPower(0);
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 24) {
                telemetry.addLine("drive off of start line");
                autoThing.driveController.driveFieldRelative(0.22, -0.33, 0); //(-0.2, -0.5, 0)
                if (getRuntime() - stateStartTime > 2.0) {//(getRuntime() - stateStartTime > 1.5)
                    state++;stateStartTime = getRuntime();
                }
            }/* else if (state == 18) {
                // shoot balls
                telemetry.addLine("Shooting balls...");
                autoThing.intake.setPower(1.9);
                autoThing.feeder.setPower(1.9);
                autoThing.feeder2.setPower(-0.4);

                if (getRuntime() - stateStartTime > 3.0) {
//                shooter.setPower(0); // don't disable shooter because...
                    autoThing.intake.setPower(0);
                    autoThing.feeder.setPower(0);
                    autoThing.feeder2.setPower(0);

                    state++;
//                state = -1; // end
                    stateStartTime = getRuntime();
                }/*

            else if (state == 7) {
                telemetry.addLine("Drive 3...");
                autoThing.driveController.driveFieldRelative(0.0, 0.125, 0);


                if (getRuntime() - stateStartTime > 2.0) {
                    state++;
//                state = -1; // end
                    stateStartTime = getRuntime();
                }
            } else if (state == 8) {
                telemetry.addLine("Drive 4...");
                autoThing.driveController.driveFieldRelative(-0.125, 0.0, 0);
                if (getRuntime() - stateStartTime > 1.25) {
                    state++;
                    //                state = -1; // end
                    stateStartTime = getRuntime();
                }
            } else if (state == 9) {
//                autoThing.driveController.driveFieldRelative(0.25, 0.0, 0);

                if (getRuntime() - stateStartTime > 0.2) {
                    state++;
                    stateStartTime = getRuntime();
                }
            } else if (state == 10) {
                // shoot balls
                telemetry.addLine("Shooting balls...");
                autoThing.intake.setPower(1.9);
                autoThing.feeder.setPower(1.9);
                autoThing.feeder2.setPower(-0.4);

                if (getRuntime() - stateStartTime > 3.0) {
                    autoThing.shooter.setPower(0);
                    autoThing.intake.setPower(0);
                    autoThing.feeder.setPower(0);
                    autoThing.feeder2.setPower(0);

                    state++;
                    stateStartTime = getRuntime();
                }
            }*/ else if (state == 25) {
                autoThing.driveController.driveFieldRelative(0.0, 0, 0);

                if (getRuntime() - stateStartTime > 1.0) { //(getRuntime() - stateStartTime > 1.5)
                    autoThing.driveController.stop();

                    state++;
                    stateStartTime = getRuntime();
                }
            }




            return new AutoStateThing(state, stateStartTime);
        });

    }

}
