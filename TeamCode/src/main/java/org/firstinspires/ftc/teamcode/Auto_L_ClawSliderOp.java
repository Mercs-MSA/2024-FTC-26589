//
// Class to handle CLAW slider operation

package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class


Auto_L_ClawSliderOp {

    // Member variables

    // member variables - initialized in the constructor

    public DcMotor clawSliderMotor;
    public DcMotor clawSliderRotationMotor;

    boolean slideForward;
    boolean slideBackward;
    boolean rotateToFront;
    boolean rotateToBack;
    boolean sliderInHoldingPosition;
    int currentRotationPosition;
    int desiredRotationPosition;
    int idleRotationPosition;

    public Gamepad  myGamePad2;
    public Telemetry    clawTelemetry;

    // Constructor
    public Auto_L_ClawSliderOp(@NonNull HardwareMap hardwareMap, Gamepad secondGamePad, Telemetry telemetry) {

        clawSliderMotor = hardwareMap.get(DcMotor.class, "claw_slider_drive");
        clawSliderRotationMotor = hardwareMap.get(DcMotor.class, "claw_slider_rotation");


        clawSliderRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // on stop, make it hold the motor

        slideForward = false;
        slideBackward = false;

        rotateToFront = false;
        rotateToBack = false;
        currentRotationPosition = 0;
        desiredRotationPosition = 0;
        idleRotationPosition = 0;

        sliderInHoldingPosition = false;

        clawSliderRotationMotor.setPower(0.0);
        clawSliderRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawSliderRotationMotor.setTargetPosition(0);

        idleRotationPosition = clawSliderRotationMotor.getCurrentPosition();

        myGamePad2 = secondGamePad;
        clawTelemetry = telemetry;
    }

    // OperateClawSlider() - Function to operate the claw slider.
    // Check in My_26589_TeamCode.java for Gampepad key assignment

    public void OperateClawSlider(int direction, int level) throws InterruptedException {

        int currentSliderPosition = 0;
        clawSliderMotor.setPower(0.0);

        clawSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawSliderMotor.setTargetPosition(0);

        clawTelemetry.addData("Slider current position 1 :  ", "%d", clawSliderMotor.getCurrentPosition());
        clawTelemetry.update();

        clawSliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clawSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (direction == 0) {               // push the slider INWARD
            slideForward = true;
            slideBackward = false;
            clawSliderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            clawTelemetry.addData("DIRECTION 0 - Level :  ", "%d", level);
        }
        else if (direction == 1) {           // pull the slider OUTWARD
            slideBackward = true;
            slideForward = false;
            clawSliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            clawTelemetry.addData("DIRECTION 1 - Level :  ", "%d", level);
        }
        else {                                      // hold the slider to current length
            slideBackward = false;
            slideForward = false;
            clawSliderMotor.setPower(0.02);
            clawTelemetry.addData("INVALID DIRECTION :  ", "%d", direction);
            Thread.sleep(3000);
            return;
        }
        clawTelemetry.update();


        int desiredPosition = 0;
        switch (level)
        {
            case 0:
                desiredPosition = 0;
                break;
            case 1:
                desiredPosition = 2500;
                break;
            case 2:
                desiredPosition = 4500  ;
                break;
            case 3:
                desiredPosition = 3500;
                break;
        }
        clawSliderMotor.setTargetPosition(desiredPosition);

        currentSliderPosition = clawSliderMotor.getCurrentPosition();
        clawTelemetry.addData("Slider current position 2:  ", "%d", currentSliderPosition);
        clawTelemetry.addData("Desired Position:  ", "%d", desiredPosition);
        clawTelemetry.update();

        clawSliderMotor.setPower(0.6);

        do{
            currentSliderPosition = clawSliderMotor.getCurrentPosition();
            clawTelemetry.addData("Slider current position 2:  ", "%d", currentSliderPosition);
            clawTelemetry.addData("Desired Position:  ", "%d", desiredPosition);
            clawTelemetry.update();
        } while(abs(currentSliderPosition) <= desiredPosition);  //while(clawSliderMotor.isBusy());

        clawSliderMotor.setPower(0.0); // stop the slider from moving any further


    } // end OperateClawSlider()

    //////////////////////////////////////////////////////////////////////////////////////
    // RotateClawSlider() - Rotate the claw slider to a desired height from current position
    public void RotateClawSlider(int level) throws InterruptedException {

        if (level < 0 || level > 6)    // invalid value
            return;

        // First get current position
        currentRotationPosition = clawSliderRotationMotor.getCurrentPosition();

        int additionalLiftfromIdle = 0;
        switch(level)
        {
            case 0: additionalLiftfromIdle = 75;     // Idle position
                break;
            case 1: additionalLiftfromIdle = 400;    // Driving level
                break;
            case 2: additionalLiftfromIdle = 1170;    // For LOWER Basket    20" high
                break;
            case 3: additionalLiftfromIdle = 1300;    // For UPPER Basket    36" high
                break;
            case 4: additionalLiftfromIdle = 500;    // For LOWER Bar       13" high
                break;
            case 5: additionalLiftfromIdle = 675;    // For UPPER Bar       26" high
                break;
            case 6: additionalLiftfromIdle = 650;
                break;
            default: // should not happen ... but HOLD position
                additionalLiftfromIdle = abs(currentRotationPosition);
                break;
        }
        desiredRotationPosition = idleRotationPosition + additionalLiftfromIdle;

        clawTelemetry.addData("Current Rotation Position :  ", "%d", clawSliderRotationMotor.getCurrentPosition());
        clawTelemetry.addData("additionalLift :  ", "%d", additionalLiftfromIdle);
        clawTelemetry.addData("Desired Position :  ", "%d", desiredRotationPosition);
        clawTelemetry.update();

        if (desiredRotationPosition > abs(currentRotationPosition)) {
            clawSliderRotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            clawSliderRotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            clawSliderRotationMotor.setPower(0.3);
            currentRotationPosition = clawSliderRotationMotor.getCurrentPosition();
            while (desiredRotationPosition > abs(currentRotationPosition))
            {
                //keep rotating until we reach desired position
                currentRotationPosition = clawSliderRotationMotor.getCurrentPosition();
                clawTelemetry.addData("Rotating REVERSE :  ", "%d", clawSliderRotationMotor.getCurrentPosition());
                clawTelemetry.addData("Desired Position :  ", "%d", desiredRotationPosition);
                clawTelemetry.update();
            }
        }
        else if (desiredRotationPosition < abs(currentRotationPosition)) {
            clawSliderRotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            clawSliderRotationMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            clawSliderRotationMotor.setPower(0.3);
            currentRotationPosition = clawSliderRotationMotor.getCurrentPosition();

            while (desiredRotationPosition < abs(currentRotationPosition))
            {
                //keep rotating until we reach desired position
                currentRotationPosition = clawSliderRotationMotor.getCurrentPosition();
                clawTelemetry.addData("Rotating FORWARD :  ", "%d", clawSliderRotationMotor.getCurrentPosition());
                clawTelemetry.addData("Desired Position :  ", "%d", desiredRotationPosition);
                clawTelemetry.update();
            }
        }
        // HOLD at desired position
        clawSliderRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clawSliderRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawSliderRotationMotor.setTargetPosition(currentRotationPosition);
        clawSliderRotationMotor.setPower(0.1);
        sliderInHoldingPosition = true;

        clawTelemetry.addData("HOLDING Position :  ", "%d", clawSliderRotationMotor.getCurrentPosition());
        clawTelemetry.addData("Desired Position :  ", "%d", desiredRotationPosition);
        clawTelemetry.update();

    } // end RotateClawSlider()


    // Call moveSliderToIdlePosition() function when the claw slider needs to get back to ies idle position
    public void moveSliderToIdlePosition()
    {
        clawSliderRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clawSliderRotationMotor.setTargetPosition(idleRotationPosition);
        clawSliderRotationMotor.setPower(0.1);
        clawSliderRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawSliderRotationMotor.setPower(0.0);
    }

} // end of class
