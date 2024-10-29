/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Our robot's Autonomous operation when placed on the LEFT side of other alliance.
 * Main functions:
 * 1. Drop the pre-sample in upper bucket
 * 2. Position the robot in appropriate orientation (and slider position) to be ready
 *    to collect samples in TELEOP mode
 * To execute, select this mode in the list of programs on the FTC Driver Station.
 *
 */

@TeleOp
// My_26589_Autonomous_L class
// - Main class for this program
//
// *********************************************************************
// ************** MAJOR  FUNCTIONS *************************************
// *********************************************************************
// When opMode is active:
//   1 Drive the robot to the location closer to the basket assembly such
//     that a sample can be dropped in the upper basket
//   2 TBD
//   3 TBD
//
// *********************************************************************
// **** 1 Drive the Robot **********************************************
// *********************************************************************
// PART - 1
//      1. Drive the robot forward                :   6 inches
//      2. Drive the robot sideways to its left   :  24 inches
//      3. Rotate the robot                       : 120 deg
//      4. Rotate slider upwards to 75 degrees
//      5. Roll the slider forward to full length
//      6. Rotate the claw 180 degrees
//      7. Open the claw  (sample drops in the upper bucket)
//      8. Drive the robot backward                :   6 inches
//      9. Drive the robot sideways to its left    :  24 inches
//      10. Roll the slider back to 25% length
//      11. Hold the slider in that location (with open claw)
//


public class My_26589_Autonomous_L extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor clawSliderMotor = null;
    private DcMotor clawSliderRotationMotor = null;

    // Define class members
    ClawServoOp myClawServoOp = null;
    ClawSliderOp_Autonomous_L myClawSliderOp = null;
    static final int    CYCLE_MS    =   20;     // period of each cycle

    @Override
    public void runOpMode() {

        // =========   Drivebase Movement ==============================================
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");


        clawSliderMotor = hardwareMap.get(DcMotor.class, "claw_slider_drive");
        clawSliderRotationMotor = hardwareMap.get(DcMotor.class, "claw_slider_rotation");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot h
        // as additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the Claw servos
        myClawServoOp = new ClawServoOp(hardwareMap, gamepad2, telemetry);
        myClawSliderOp = new ClawSliderOp_Autonomous_L(hardwareMap, gamepad2, telemetry);
        waitForStart();
        runtime.reset();
        int direction = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // set power for all motors to same value
            double wheelMotorPower = 0.1;

            sleep(2000);

            // move forward
            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

            wheelMotorPower = 0.25;
            leftFrontDrive.setPower(wheelMotorPower);
            rightFrontDrive.setPower(wheelMotorPower);
            leftBackDrive.setPower(wheelMotorPower);
            rightBackDrive.setPower(wheelMotorPower);


            //stop
            sleep(2000);
            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFrontDrive.setPower(0.0);
            rightFrontDrive.setPower(0.0);
            leftBackDrive.setPower(0.0);
            rightBackDrive.setPower(0.0);

            sleep(5000);

            // rotate ROBOT
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

            // Send calculated power to wheels
            wheelMotorPower = 0.25;

            leftFrontDrive.setPower(wheelMotorPower);
            rightFrontDrive.setPower(wheelMotorPower);
            leftBackDrive.setPower(wheelMotorPower);
            rightBackDrive.setPower(wheelMotorPower);


            // =========   Claw slider movement ==============================================
            // Code to control the claw slider forward and back
//            myClawSliderOp.OperateClawSlider();
            myClawSliderOp.RotateClawSlider(direction);
            sleep(3000);
            myClawSliderOp.HoldClawSliderAtCurrentRotationPosition();
            if (direction == 1)
                direction = 0;
            else direction = 1;
            // =========   Claw slider movement  - END ==============================================

            // =========   Claw movement ==============================================

            clawSliderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            clawSliderMotor.setPower(0.4);

            sleep(1000);

            clawSliderMotor.setPower(0.0);



            // Operate claw - it check if claw needs to be moved, and operates if needed
            myClawServoOp.setOpenClaw();
            // =========   Claw movement  - END ==============================================
/*
            sleep(1000);
            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFrontDrive.setPower(0.0);
            rightFrontDrive.setPower(0.0);
            leftBackDrive.setPower(0.0);
            rightBackDrive.setPower(0.0);
*/
            idle();
        }
        // stop the slider from abruptly falling
        telemetry.update();
    }
}
