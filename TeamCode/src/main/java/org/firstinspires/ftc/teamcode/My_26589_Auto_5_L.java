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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous
// My_26589_Auto_L_orig_claw class
// - Main class for this program
//
// *********************************************************************
// ************** MAJOR  FUNCTIONS *************************************
// *********************************************************************
// When opMode is active:
//   1 Drive the robot to the location closer to the basket assembly such
//     that a sample can be dropped in the upper basket
//   2 Rotate and pick up one of the spike samples from the floor and
//     drop it in the LOWER basket
//

/////////////   uses NEW claw ///////////////////////////////////////



public class My_26589_Auto_5_L extends LinearOpMode {

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

        int x = 0;
        double position = 0.0;

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

// *********************************************************************
// **** 1 Drive the Robot **********************************************
// *********************************************************************
// STEPS
//      //// The robot should be placed properly sideways, claw pointing
//      //// towards the net zone.
//      In autonoumous mode, we will drop a pre-loaded sample in UPPER basket,
//      pickup another sample from floor, and place it in LOWER basket
//
//      1. Lift the clew slider bar slightly
//      2. Rotate claw upwards appropriately
//      3. Drive the robot forward to appropriate distance
//      4. Rotate the robot appropriately such that the claw points toward center of basket
//      5. Rotate the claw slider fully upwards
//      6. Elongate the claw slider bar to reach UPPER basket
//      7. Rotate claw downwards appropriately such that the sample drops
//      8. Reduce length of claw slider
//      9. Rotate and reach to pick up sample from floor
//     10. Rotate again and place sample in LOWER basket


            myClawSliderOp.RotateClawSlider(3);

            driveRobotStraight(moveDirection.FORWARD, encoderResolution*19.00); // 9 inches

            myClawSliderOp.OperateClawSlider(1, 2);

            myClawServoOp.RotateClaw(0.9);

            rotateRobot(10, rotation.COUNTER_CLOCKWISE);

            driveRobotStraight(moveDirection.FORWARD, encoderResolution*9.00);

            myClawSliderOp.RotateClawSlider(2);

            sleep(1000);

            myClawServoOp.openClaw();            // SAMPLE dropped in UPPER basket  //////////

            myClawSliderOp.RotateClawSlider(3);

            myClawSliderOp.OperateClawSlider(0, 3);

            driveRobotStraight(moveDirection.REVERSE, encoderResolution*3.00);

            rotateRobot(135, rotation.CLOCKWISE);

            myClawServoOp.openClaw();
            myClawServoOp.openClaw();

            driveRobotStraight(moveDirection.FORWARD, encoderResolution*8.00);

            myClawSliderOp.RotateClawSlider(0);

            myClawServoOp.closeClaw();
            myClawServoOp.closeClaw();
            myClawServoOp.closeClaw();

            sleep(1000);

            myClawSliderOp.RotateClawSlider(3);

            driveRobotStraight(moveDirection.REVERSE, encoderResolution*10.00);

            rotateRobot(210, rotation.COUNTER_CLOCKWISE);

            driveRobotStraight(moveDirection.FORWARD, encoderResolution*3.00);

            myClawSliderOp.RotateClawSlider(6);

            sleep(500);

            myClawServoOp.openClaw();        // SAMPLE dropped in LOWER basket
            myClawServoOp.openClaw();

            // back away from the basket and lower the slider
            driveRobotStraight(moveDirection.REVERSE, encoderResolution*8.00);
            myClawSliderOp.RotateClawSlider(0);

            idle();
            sleep(30000);
            break;
        }
        telemetry.update();
    }
    public void rotateRobot(int degrees, rotation directionToRotate)
    {

        // this functionality is written using time - very crude way
        // right way is to use a sensor which provides live rotation feedback.

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


        setDrivePower(0.3);

        switch(degrees)
        {
            case 10:
                sleep(1300);
                break;
            case 135:
                sleep(2300);

                break;
            case 180:
                sleep(3500);
                break;
            case 210:
                sleep(2200);
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

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Desired Distance", "%d", (int)desiredDistance);
        telemetry.addData("Encoder Resolution", "%d", (int)encoderResolution);
        telemetry.addData("Drive Distance", "%d", leftBackDrive.getCurrentPosition());
        telemetry.update();

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
