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
/////////////////////////////////////////////////////////////////////////////////////////////
 * Our robot's Autonomous operation when placed on the LEFT side of other alliance.
 * Main functions:
 * 1. Drive to the alliance basket
 * 2. Drop the pre-sample in upper bucket
 * 3. Position the robot near the submersible in appropriate orientation (and slider position) to be ready
 *    to collect samples in TELEOP mode
 * To execute, select this mode in the list of programs on the FTC Driver Station.
/////////////////////////////////////////////////////////////////////////////////////////////
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
// STEPS
//      1. Lift the clew slider bar to a level to place specimen on higher bar
//      2. Rotate claw upwards appropriately
//      3. Drive the robot forward                :   10 inches
//      4. Elongate the claw slider bar to reach top bar
//      5. Rotate claw downwards approprately such that the specimen gets inserted on the bar
//      6. Reduce length of claw slider to idle
//      7. Drive robot sideways to reach NET ZONE   :  24 inches
//      8. Rotate claw slider bar to idle position
//      9. STOP the program


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
    Auto_L_ClawServoOp myClawServoOp = null;
    Auto_L_ClawSliderOp myClawSliderOp = null;
    static final int    CYCLE_MS    =   20;     // period of each cycle
    int direction = 0;

    enum rotation
    {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    };
    enum moveDirection
    {
        FORWARD,
        REVERSE
    };

    enum sliderLength
    {
        IDLE,
        MIDDLE,
        HIGH
    };
    int encoderResolution = 35;  // encoder reading per inch (385 ppr / 11 inch wheel circumference)


    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the Claw mechanism
        myClawServoOp = new Auto_L_ClawServoOp(hardwareMap, gamepad2, telemetry);
        myClawSliderOp = new Auto_L_ClawSliderOp(hardwareMap, gamepad2, telemetry);

        // =========   Initialise Drivebase Movement ==============================================
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        //************************************************************
        // Use leftBackDrive motor for distance measurement
        //************************************************************
        leftBackDrive.setPower(0.0);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setTargetPosition(0);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Drive Distance", "%d", leftBackDrive.getCurrentPosition());
        telemetry.setMsTransmissionInterval(200);
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //////   STEP 1   //////////////////////////////////////////////////////////////////////
            // Get the claw slider in desired position to place SPECIMEN in UPPER bar
            myClawSliderOp.RotateClawSlider(5);
//            sleep(10000);
/*

            //////   STEP 2   //////////////////////////////////////////////////////////////////////
            myClawServoOp.RotateClaw(0.8);
            sleep(5000);
            myClawServoOp.RotateClaw(0.2);
            sleep(5000);
            myClawServoOp.RotateClaw(0.8);
            sleep(5000);
            myClawServoOp.RotateClaw(0.2);
*/


            //////////////////////////////////////////////////////////////////////////////////////
            // Drive the robot few (~15) inches
            // Circumference of wheel is 300mm
            // Encoder resolution is 1425.1 PPR
            // One rotation = 301mm = 11.85 inches
            // 1 inch = 120.25 encoder parts

            driveRobotStraight(moveDirection.FORWARD, encoderResolution*18.0); // 6 inches


            //////////////////////////////////////////////////////////////////////////////////////
            //  Rotate robot 90 degrees

//            rotateRobot(120, rotation.COUNTER_CLOCKWISE);

            // Lift the claw slider to set it up for sample to be dropped in upper basket
            //myClawSliderOp.RotateClawSlider(4);
/*
            // Slide the claw closer to the basket
            myClawSliderOp.OperateClawSlider(0, 2);

            // OPEN the claw - drop the sample in the Upper basket
            myClawServoOp.setOpenClaw();

            // Slide the slider to middle position
            myClawSliderOp.OperateClawSlider(1, 1);

            // rotate robot so it is parallel to wall
            rotateRobot(30, rotation.CLOCKWISE);

            // drive robot to PARK position
            driveRobotStraight(moveDirection.REVERSE, encoderResolution*84.0); // 84 inches
*/
            idle();
            sleep(50000);
            myClawSliderOp.RotateClawSlider(0);
            sleep(10000);

            break;
        }
        // stop the slider from abruptly falling
        telemetry.update();
    }
    public void rotateRobot(int degrees, rotation directionToRotate)
    {
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (directionToRotate == rotation.COUNTER_CLOCKWISE) {
            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        }
        else {
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        }

        setDrivePower(0.2);

        switch(degrees)
        {
            case 90:
                sleep(1000);
                break;
            case 120:
                sleep(1500);
                break;
            case 180:
                sleep(2000);
                break;
            case 210:
                sleep(2500);
                break;
            case 240:
                sleep(3000);
                break;
            case 270:
                sleep(3500);
                break;
            default:
                break;
        }
        setDrivePower(0.0);
    }

    public void driveRobotStraight(moveDirection dDir, double desiredDistance) {

        setDrivePower(0.0);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.update();

        if (dDir == moveDirection.FORWARD) {
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        }
        else if (dDir == moveDirection.REVERSE) {
            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        }
        else
            return; // invalid direction

        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setTargetPosition((int)desiredDistance);

        setDrivePower(0.2);

        //keep rotating until we reach desired position
        while (leftBackDrive.getCurrentPosition() < desiredDistance)
        {
            // display status
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Desired Distance", "%d", (int)desiredDistance);
            telemetry.addData("Encoder Resolution", "%d", (int)encoderResolution);
            telemetry.addData("Drive Distance", "%d", leftBackDrive.getCurrentPosition());
            telemetry.update();
        }

        // stop the robot
        setDrivePower(0.0);
    }
    public void setDrivePower(double power) {
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        if (power == 0.0)
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
