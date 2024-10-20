//
// Class to handle CLAW slider operation

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClawSliderOp {

    // Member variables

    // member variables - initialized in the constructor

    public DcMotor clawSliderMotor;
    public DcMotor clawSliderRotationMotor;

    boolean slideForward;
    boolean slideBackward;
    boolean rotateToFront;
    boolean rotateToBack;
    int currentRotationPosition;

    public Gamepad  myGamePad2;
    public Telemetry    clawTelemetry;

    // Constructor
    public ClawSliderOp(@NonNull HardwareMap hardwareMap, Gamepad secondGamePad, Telemetry telemetry) {

        clawSliderMotor = hardwareMap.get(DcMotor.class, "claw_slider_drive");
        clawSliderRotationMotor = hardwareMap.get(DcMotor.class, "claw_slider_rotation");

        slideForward = false;
        slideBackward = false;

        rotateToFront = false;
        rotateToBack = false;
        currentRotationPosition = 0;
        clawSliderRotationMotor.setPower(0.0);
        clawSliderRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawSliderRotationMotor.setTargetPosition(0);


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

        RotateClawSlider();
    } // end OperateClawSlider()

    // RotateClawSlider() - Function to rotate the claw slider.
    // Check in My_26589_TeamCode.java for Gampepad key assignment

    public void RotateClawSlider() {
        //
        if (myGamePad2.dpad_up) {               // Rotate towards front of robot
            rotateToFront = true;
            rotateToBack = false;

            clawSliderRotationMotor.setPower(0.2);
            clawSliderRotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            clawSliderRotationMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            currentRotationPosition = clawSliderRotationMotor.getCurrentPosition();

        }
        else if (myGamePad2.dpad_down) {       // Rotate towards back of robot
            rotateToBack = true;
            rotateToFront = false;

            clawSliderRotationMotor.setPower(0.2);
            clawSliderRotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            clawSliderRotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            currentRotationPosition = clawSliderRotationMotor.getCurrentPosition();

        }
        else {                                  // HOLD the slider steady
            rotateToFront = false;
            rotateToBack = false;
            clawSliderRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            clawSliderRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            clawSliderRotationMotor.setTargetPosition(currentRotationPosition);
            clawSliderRotationMotor.setPower(0.1);
        }
        clawTelemetry.addData("Rotation Position :  ", "%d", clawSliderRotationMotor.getCurrentPosition());

    } // end RotateClawSlider()

} // end of class
