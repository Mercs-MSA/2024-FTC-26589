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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


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
// My_FieldCentricTest_Drive class
// - Main class for this program to drive in Field Centric coordinates
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

public class My_FieldCentricTest_Drive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Define class members
    ClawServoOp myClawServoOp = null;
    ClawSliderOp myClawSliderOp = null;
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

        IMU imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                ));
        imu.initialize(parameters);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Initialize the Claw servos
        myClawServoOp = new ClawServoOp(hardwareMap, gamepad2, telemetry);
        myClawSliderOp = new ClawSliderOp(hardwareMap, gamepad2, telemetry);
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double ly = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value  // axial
            double lx = gamepad1.left_stick_x;   // lateral
            double rx = gamepad1.right_stick_x; // yaw

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = ly + lx + rx;
            double rightFrontPower = ly - lx - rx;
            double leftBackPower = ly - lx + rx;
            double rightBackPower = ly + lx - rx;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);


            double power = 0.2 + (0.6 * gamepad1.right_trigger);
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double adjustedLx = -ly * Math.sin(heading) + lx * Math.cos(heading);
            double adjustedLy = ly * Math.cos(heading) + lx  * Math.sin(heading);


            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }


            heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(((adjustedLy + adjustedLx + rx) / max) * power);
            rightFrontDrive.setPower(((adjustedLy - adjustedLx - rx) / max) * power);
            leftBackDrive.setPower(((adjustedLy - adjustedLx + rx) / max) * power);
            rightBackDrive.setPower(((adjustedLx + adjustedLy - rx) / max) * power);

            // Show the elapsed game time and wheel power.
/*            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
*/
        }
    }
}
