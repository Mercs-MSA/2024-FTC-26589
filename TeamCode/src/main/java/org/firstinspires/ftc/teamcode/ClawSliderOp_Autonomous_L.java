//
// Class to handle CLAW slider operation

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.EnumMap;

public class


ClawSliderOp_Autonomous_L {

    // Member variables

    // member variables - initialized in the constructor

    public DcMotor clawSliderMotor;
    public DcMotor clawSliderRotationMotor;

    boolean slideForward;
    boolean slideBackward;
    boolean rotateToFront;
    boolean rotateToBack;
    int currentRotationPosition;
    int idleRotationPosition;

    public Gamepad  myGamePad2;
    public Telemetry    clawTelemetry;

    // Constructor
    public ClawSliderOp_Autonomous_L(@NonNull HardwareMap hardwareMap, Gamepad secondGamePad, Telemetry telemetry) {

        clawSliderMotor = hardwareMap.get(DcMotor.class, "claw_slider_drive");
        clawSliderRotationMotor = hardwareMap.get(DcMotor.class, "claw_slider_rotation");


        clawSliderRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // on stop, make it hold the motor

        slideForward = false;
        slideBackward = false;

        rotateToFront = false;
        rotateToBack = false;
        currentRotationPosition = 0;
        idleRotationPosition = 0;

        clawSliderRotationMotor.setPower(0.0);
        clawSliderRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawSliderRotationMotor.setTargetPosition(0);

        idleRotationPosition = clawSliderRotationMotor.getCurrentPosition();

        myGamePad2 = secondGamePad;
        clawTelemetry = telemetry;
    }

    // OperateClawSlider() - Function to operate the claw slider.
    // Check in My_26589_TeamCode.java for Gampepad key assignment

    public void OperateClawSlider() {
    //
        if (myGamePad2.right_bumper) {               // push the slider outward
            slideForward = true;
            slideBackward = false;

            clawSliderMotor.setPower(0.4);
            clawSliderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else if (myGamePad2.left_bumper) {           // pull the slider inward
            slideBackward = true;
            slideForward = false;

            clawSliderMotor.setPower(0.4);
            clawSliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else {                                      // hold the slider to current length
            slideBackward = false;
            slideForward = false;
            clawSliderMotor.setPower(0.0);
        }

       // RotateClawSlider(1);
    } // end OperateClawSlider()

    // RotateClawSlider(int timeSec) - Function to rotate the claw slider
    // timeSec : Time in seconds to rotate and then lock at that position
    // direction : Direction in which the slider should rotate
    //             0 = FORWARD     1 = BACKWARD
    // Check in My_26589_TeamCode.java for Gampepad key assignment

    public void RotateClawSlider(int direction) {
        //
        if (direction == 0) {               // Rotate towards front of robot
            rotateToFront = true;
            rotateToBack = false;

            clawSliderRotationMotor.setPower(0.2);
            clawSliderRotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            clawSliderRotationMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            currentRotationPosition = clawSliderRotationMotor.getCurrentPosition();

        } else if (direction == 1) {          // Rotate towards back of robot
            rotateToBack = true;
            rotateToFront = false;

            clawSliderRotationMotor.setPower(0.2);
            clawSliderRotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            clawSliderRotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            currentRotationPosition = clawSliderRotationMotor.getCurrentPosition();

        }
    }

    public void HoldClawSliderAtCurrentRotationPosition() {

        // HOLD the slider steady
        rotateToFront = false;
        rotateToBack = false;
        clawSliderRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            clawSliderRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawSliderRotationMotor.setTargetPosition(currentRotationPosition);
        clawSliderRotationMotor.setPower(0.1);
        }

    // Call moveSliderToIdlePosition() function when the claw slider needs to get back to ies idle position
    public void moveSliderToIdlePosition() {
        clawSliderRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clawSliderRotationMotor.setTargetPosition(idleRotationPosition);
        clawSliderRotationMotor.setPower(0.1);
        clawSliderRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

} // end of class
