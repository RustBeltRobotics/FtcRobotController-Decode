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
@Autonomous(name = "!_RedShoot9Better", group = "Robot")
public class AutoRedShoot6New2 extends LinearOpMode {

    static final double R60 = Math.toRadians(60);

    double[][] moves = {
            //  { Forward,  Right,   rot,  time }
            {  0.0,    0.0,   0.0,  0.5   },  // 0  - Shooter spin-up
            {  0.25,   0.0,   0.0,  0.2   },  // 1  - Drive backwards off of goal position
            {  0.0,    0.0,   0.0,  2.3   },  // 2  - Wait for shooter to spin up
            {  0.0,    0.0,   0.0,  3.0   },  // 3  - Shooting balls
            {  0.25,   0.0,   0.0,  0.75  },  // 4  - Drive Away from goal towards first row pickup spot
            {  0.22,  -0.3,   0.0,  0.25  },  // 5  - Drive Away from goal towards 1st row pickup position 2
            {  0.0,    0.0,   R60,  0.28  },  // 6  - Rotate to pickup balls
            { -0.125, -0.125, 0.0,  1.7   },  // 7  - Strafe to pickup fir6st row balls
            {  0.125,  0.125, 0.0,  1.7   },  // 8  - Strafe back to first row start position
            {  0.0,    0.0,  -R60,  0.22  },  // 9  - Rotate back to face goal again
            { -0.22,   0.3,   0.0,  0.01  },  // 10 - Drive Away from first row pickup towards 2nd row pickup
            { -0.25,   0.0,   0.0,  1.2   },  // 11 - Drive towards goal
            {  0.25,   0.0,   0.0,  0.2   },  // 12 - Drive away from goal slightly
            {  0.0,    0.0,   0.0,  3.5   },  // 13 - Shooting balls
            {  0.25,   0.0,   0.0,  0.9   },  // 14 - Drive Away from goal towards first row pickup spot
            {  0.22,  -0.3,   0.0,  0.8   },  // 15 - Drive Away from goal towards 2nd row pickup
            {  0.0,    0.0,   R60,  0.3   },  // 16 - Rotate to pickup balls
            { -0.125, -0.125, 0.0,  1.8   },  // 17 - Strafe to pickup second row balls
            {  0.125,  0.125, 0.0,  0.8   },  // 18 - Strafe back to initial pickup position second row balls
            {  0.0,    0.0,  -R60,  0.35  },  // 19 - Rotate back to face goal again
            { -0.22,   0.33,  0.0,  1.55  },  // 20 - Drive towards 1st move position
            { -0.25,   0.0,   0.0,  0.6   },  // 21 - Drive into goal and prepare to shoot
            {  0.25,   0.0,   0.0,  0.2   },  // 22 - Drive away from goal slightly
            {  0.0,    0.0,   0.0,  4.5   },  // 23 - Shooting balls
            {  0.22,  -0.33,  0.0,  2.0   },  // 24 - Drive off of start line
            {  0.0,    0.0,   0.0,  1.0   },  // 25 - (stop)
    };

    double[] mirror = { 1.0, -1.0, -1.0, 1.0 };

    BasicAutoClass autoThing = new BasicAutoClass();

    @Override
    public void runOpMode() {
        // blue shoot, aquire and shoot
        for (int i = 0; i < moves.length; i++) {
            for (int j = 0; j < moves[i].length; j++) {
                moves[i][j] *= mirror[j];
            }
        }

        autoThing.onRunOpMode(this, (Integer state, Double stateStartTime) -> {

            if (state == 0) {
                telemetry.addLine("Shooter spin-up...");
                autoThing.shooter.setPower(-0.55);
                autoThing.driveController.driveFieldRelative(moves[0][0], moves[0][1], moves[0][2]);
                if (getRuntime() - stateStartTime > moves[0][3]) {state++;stateStartTime = getRuntime();
                }
            } else if (state == 1) {
                telemetry.addLine("Drive backwards off of goal position");
                autoThing.driveController.driveFieldRelative(moves[1][0], moves[1][1], moves[1][2]);
                if (getRuntime() - stateStartTime > moves[1][3]) { state++; stateStartTime = getRuntime();
                }
            } else if (state == 2) {
                telemetry.addLine("Wait for shooter to spin up");
                if (getRuntime() - stateStartTime > moves[2][3]) {state++;stateStartTime = getRuntime();
                }
            } else if (state == 3) {
                telemetry.addLine("Shooting balls...");
                autoThing.intake.setPower(1.9);autoThing.feeder.setPower(1.9);autoThing.feeder2.setPower(-0.4);
                if (getRuntime() - stateStartTime > moves[3][3]) {
                    autoThing.intake.setPower(0);autoThing.feeder.setPower(0);autoThing.feeder2.setPower(0);
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 4) {
                telemetry.addLine("Drive Away from goal towards first row pickup spot");
                autoThing.driveController.driveFieldRelative(moves[4][0], moves[4][1], moves[4][2]);
                if (getRuntime() - stateStartTime > moves[4][3]) { //0.8
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 5) {
                telemetry.addLine("Drive Away from goal towards 1nd row pickup posiiton 2");
                autoThing.driveController.driveFieldRelative(moves[5][0], moves[5][1], moves[5][2]);//(-0.15, -0.5, 0) //tried to move back more and left less.
                if (getRuntime() - stateStartTime > moves[5][3]) {//(getRuntime() - stateStartTime > 2.0)
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 6) {
                telemetry.addLine("Rotate to pickup balls");
                autoThing.driveController.driveFieldRelative(moves[6][0], moves[6][1], moves[6][2]);
                if (getRuntime() - stateStartTime > moves[6][3]) { //0.32
                    state++; stateStartTime = getRuntime();
                }
            } else if (state == 7) {
                telemetry.addLine("Strafe to pickup first row balls");
                autoThing.intake.setPower(1.9);
                autoThing.feeder.setPower(1.9);
                autoThing.driveController.driveFieldRelative(moves[7][0], moves[7][1], moves[7][2]); //(-0.125, -0.125, 0)
                if (getRuntime() - stateStartTime > moves[7][3]) { //1.95
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 8) {
                telemetry.addLine("Strafe to back to first row start position");
                autoThing.intake.setPower(0);
                autoThing.feeder.setPower(0);
                autoThing.driveController.driveFieldRelative(moves[8][0], moves[8][1], moves[8][2]); //(0.125, 0.125, 0)
                if (getRuntime() - stateStartTime > moves[8][3]) { //1.95
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 9) {

                telemetry.addLine("Rotate back to face goal again.");
                autoThing.driveController.driveFieldRelative(moves[9][0], moves[9][1], moves[9][2]);
                if (getRuntime() - stateStartTime > moves[9][3]) { //0.26)
                    state++;stateStartTime = getRuntime();
                }
            }
            else if (state == 10) {
                telemetry.addLine("Drive Away from first row pickup towards 2nd row pickup");
                autoThing.driveController.driveFieldRelative(moves[10][0], moves[10][1], moves[10][2]); //(-0.15, -0.5, 0) //tried to move back more and left less.
                if (getRuntime() - stateStartTime > moves[10][3]) {//0.25
                    state++;stateStartTime = getRuntime();
                }
            }else if (state == 11) {
                telemetry.addLine("Drive towards goal");
                autoThing.driveController.driveFieldRelative(moves[11][0], moves[11][1], moves[11][2]); //(0.125, 0.0, 0)
                if (getRuntime() - stateStartTime > moves[11][3]) {//(getRuntime() - stateStartTime > 2.0)
                    state++; stateStartTime = getRuntime();
                }
            }else if (state == 12) {
                telemetry.addLine("drive away from goal slightly");
                autoThing.driveController.driveFieldRelative(moves[12][0], moves[12][1], moves[12][2]);
                if (getRuntime() - stateStartTime > moves[12][3]) { state++;stateStartTime = getRuntime();
                }
            } else if (state == 13) {
                telemetry.addLine("Shooting balls...");
                autoThing.intake.setPower(1.9);autoThing.feeder.setPower(1.9);autoThing.feeder2.setPower(-0.4);
                if (getRuntime() - stateStartTime > moves[13][3]) {
                    autoThing.intake.setPower(0);autoThing.feeder.setPower(0);autoThing.feeder2.setPower(0);
                    state++; stateStartTime = getRuntime();
                }
            }
            /***********************start of 2nd row pickup  *************************************************************/
            else if (state == 14) {
                telemetry.addLine("Drive Away from goal towards first row pickup spot");
                autoThing.driveController.driveFieldRelative(moves[14][0], moves[14][1], moves[14][2]); //(0.125, 0.0, 0)
                if (getRuntime() - stateStartTime > moves[14][3]) {//(getRuntime() - stateStartTime > 2.0)
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 15) {
                telemetry.addLine("Drive Away from goal towards 2nd row pickup");
                autoThing.driveController.driveFieldRelative(moves[15][0], moves[15][1], moves[15][2]); //(-0.15, -0.5, 0) //tried to move back more and left less.
                if (getRuntime() - stateStartTime > moves[15][3]) {//1.25
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 16) {
                telemetry.addLine("Rotate to pickup balls");
                autoThing.driveController.driveFieldRelative(moves[16][0], moves[16][1], moves[16][2]);
                if (getRuntime() - stateStartTime > moves[16][3]) {state++;stateStartTime = getRuntime(); //0.35
                }
            } else if (state == 17) {
                telemetry.addLine("Strafe to pickup second row balls");
                autoThing.driveController.driveFieldRelative(moves[17][0], moves[17][1], moves[17][2]); //(-0.125, -0.125, 0)
                autoThing.intake.setPower(1.9);
                autoThing.feeder.setPower(1.9);
                if (getRuntime() - stateStartTime > moves[17][3]) {//1.3
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 18) {
                telemetry.addLine("Strafe back to initial pickup position second row balls");
                autoThing.driveController.driveFieldRelative(moves[18][0], moves[18][1], moves[18][2]); //(-0.125, -0.125, 0)
                autoThing.intake.setPower(0);
                autoThing.feeder.setPower(0);
                if (getRuntime() - stateStartTime > moves[18][3]) { //0.8
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 19) {
                telemetry.addLine("Rotate back to face goal again.");
                autoThing.driveController.driveFieldRelative(moves[19][0], moves[19][1], moves[19][2]);
                if (getRuntime() - stateStartTime > moves[19][3]) { //0.35
                    state++; stateStartTime = getRuntime();
                }
            } else if (state == 20) {
                telemetry.addLine("Drive towards 1st move position");
                autoThing.driveController.driveFieldRelative(moves[20][0], moves[20][1], moves[20][2]); //(-0.2, -0.5, 0)
                if (getRuntime() - stateStartTime > moves[20][3]) {//1.75
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 21) {
                telemetry.addLine("Drive into goal and prepare to shoot");
                autoThing.driveController.driveFieldRelative(moves[21][0], moves[21][1], moves[21][2]); //(-0.125, 0.0, 0)
                if (getRuntime() - stateStartTime > moves[21][3]) { //0.9
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 22) {
                telemetry.addLine("drive away from goal slightly");
                autoThing.driveController.driveFieldRelative(moves[22][0], moves[22][1], moves[22][2]);
                if (getRuntime() - stateStartTime > moves[22][3]) { state++;stateStartTime = getRuntime();
                }
            }
            else if (state == 23) {
                telemetry.addLine("Shooting balls...");
                autoThing.intake.setPower(1.9);autoThing.feeder.setPower(1.9);autoThing.feeder2.setPower(-0.4);
                if (getRuntime() - stateStartTime > moves[23][3]) {
                    autoThing.intake.setPower(0);autoThing.feeder.setPower(0);autoThing.feeder2.setPower(0);
                    state++;stateStartTime = getRuntime();
                }
            } else if (state == 24) {
                telemetry.addLine("drive off of start line");
                autoThing.driveController.driveFieldRelative(moves[25][0], moves[25][1], moves[25][2]); //(-0.2, -0.5, 0)
                if (getRuntime() - stateStartTime > moves[24][3]) {//(getRuntime() - stateStartTime > 1.5)
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
