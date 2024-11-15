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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;   //changed
import com.qualcomm.robotcore.hardware.IMU;    //changed
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ClawServoOp;
import org.firstinspires.ftc.teamcode.ClawSliderOp;


// --------------- Our robot's OpMode operation ----------------


//                       Key assignments
//  |-----------------------------|-----------------------------|
//  |          GAMEPAD 1          |         GAMEPAD 2           |
//  |-----------------------------|-----------------------------|
//  |  LAx0  |   Drive            |  LA-y  |  Claw Slider UP/DN |
//  |  LAx1  |   Drive            |  LA-x  |                    |
//  |  RAx0  |   Drive            |  RA-y  |  Claw Slider Rot   |
//  |  RAx1  |   Drive            |  RA-x  |                    |
//  |    A   |                    |    A   |  Claw rotate DN    |
//  |    B   |                    |    B   |  Claw rotate UP    |
//  |    X   |  Claw OPEN         |    X   |                    |
//  |    Y   |  Claw CLOSE        |    Y   |  Claw OPEN         |
//  |   LB   |                    |   LB   |                    |
//  |   RB   |                    |   RB   |                    |
//  |   LT   |                    |   LT   |                    |
//  |   RT   |                    |   RT   |                    |
//  |  Hat   |                    |  Hat   |                    |
//  |-----------------------------|-----------------------------|


@TeleOp
// My_26589_TeamCode class
// - Main class for this program
//
// *********************************************************************
// ************** MAJOR  FUNCTIONS *************************************
// *********************************************************************
// When opMode is active:
//   1 Drive the robot
//   2 Call function to operate the claw's slider
//   3 Call function to operate the claw
//
// *********************************************************************
// **** 1 Drive the Robot **********************************************
// *********************************************************************
// * 1) Axial:    Driving forward and backward                Left-joystick Forward/Backward
// * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
// * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
//
// *********************************************************************
// **** 2 Operate the claw slider **************************************
// *********************************************************************
// Call OperateClawSlider()
// Implementation in ClawSliderOp class
//
//
// *********************************************************************
// **** 3 Operate the claw *********************************************
// *********************************************************************
// Call OperateClaw()
// Implementation in ClawServoOp class
//

public class My_26589_TeamCode extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Define class members
    ClawServoOp myClawServoOp = null;
    ClawSliderOp myClawSliderOp = null;
    static final int    CYCLE_MS    =   10;     // period of each cycle

    @Override
    public void runOpMode() {

        // =========   Drivebase Movement ==============================================
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

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
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        IMU imu = hardwareMap.get(IMU.class,"imu");   //changed next 4 lines
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the Claw mechanism
        myClawServoOp = new ClawServoOp(hardwareMap, gamepad2, telemetry);
        myClawSliderOp = new ClawSliderOp(hardwareMap, gamepad2, telemetry);
        waitForStart();
        runtime.reset();

        double maxDrivePower = 0.7;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

//          lateral = -axial * Math.sin(heading) + lateral * Math.cos(heading);
//          axial = axial * Math.cos(heading) + lateral  * Math.sin(heading);
//telemetry.addData("heading",Math.toDegrees(heading));

            // Combine the joystick requests for each axis-motion to determine
            // ach wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            // control the MAX drive speed
            if (gamepad1.right_bumper)
                maxDrivePower = 0.2;            // slow motion
            else if (gamepad1.left_bumper)
                maxDrivePower = 0.7;            // max speed

            else maxDrivePower = 0.5;           // normal speed

            if (max > maxDrivePower) {

                if (leftFrontPower < 0.0)
                    leftFrontPower  = -maxDrivePower;
                else
                    leftFrontPower  = maxDrivePower;

                if (rightFrontPower < 0.0)
                    rightFrontPower  = -maxDrivePower;
                else
                    rightFrontPower  = maxDrivePower;


                if (leftBackPower < 0.0)
                    leftBackPower  = -maxDrivePower;
                else
                    leftBackPower  = maxDrivePower;


                if (rightBackPower < 0.0)
                    rightBackPower  = -maxDrivePower;
                else
                    rightBackPower  = maxDrivePower;

/*
                leftFrontPower  = maxDrivePower;
                rightFrontPower = maxDrivePower;
                leftBackPower   = maxDrivePower;
                rightBackPower  = maxDrivePower;
*/
            }

/*
            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
*/

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Max",maxDrivePower);
            telemetry.update();


            // =========   Claw slider movement ==============================================

            // Code to control the claw slider forward and back
            myClawSliderOp.OperateClawSlider();


            // =========   Claw OPEN/CLOSE ==============================================

            // Operate claw - it check if claw needs to be moved, and operates if needed
            if(gamepad1.y) {                            // CLOSE the claw
                myClawServoOp.closeClaw = true;
                myClawServoOp.openClaw = false;
            }
            else if(gamepad1.x) {                       // OPEN the claw
                myClawServoOp.closeClaw = false;
                myClawServoOp.openClaw = true;
            }
            else {
                myClawServoOp.closeClaw = false;
                myClawServoOp.openClaw = false;
            }
            myClawServoOp.clawOpenClose();

            // =========   Claw OPEN/CLOSE ==============================================

            if(gamepad1.a) {
                myClawServoOp.rotateToFront = true;
                myClawServoOp.rotateToBack = false;
            }
            if(gamepad1.b) {
                myClawServoOp.rotateToBack = true;
                myClawServoOp.rotateToFront = false;
            }
            myClawServoOp.RotateClaw();

            sleep(CYCLE_MS);   // could be removed ?
            idle();            // could be removed ?

        }

        // TeamCode application above to terminate
        // stop the claw slider from abruptly falling
        myClawSliderOp.moveSliderToIdlePosition();
        telemetry.addData("Status", "Program stopped. All systems in idle mode.");
        telemetry.update();
    }
}
