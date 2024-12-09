//
// Class to handle all operations related to the INTAKE claw

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// This class contains code to control the claw's rotation as well as
// control the close/open operation of the claw
public class ClawServoOp {

    // Member variables
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // member variables - initialized in the constructor
    // Rotation Servo /////////////////////////////
    double positionOfRotation;
    boolean rotateToFront;
    boolean rotateToBack;
    public Servo clawRotationServo;

    // Claw servo /////////////////////////////////
    boolean closeClaw;
    boolean openClaw;
    public CRServo clawServoR;
    public CRServo clawServoL;


    // misc ///////////////////////////////////////////
    public Gamepad  myGamePad2;
    public Telemetry    clawTelemetry;

    /////////////////////////////////////////////   Constructor   /////////////////////////
    public ClawServoOp(@NonNull HardwareMap hardwareMap, Gamepad secondGamePad, Telemetry telemetry) {
        // claw initialization
        clawRotationServo = hardwareMap.get(Servo.class, "claw_Rotation"); // servo 0- CONTROL HUB
        clawServoR = hardwareMap.get(CRServo.class, "claw_R"); // servo 1 - EXPANSION HUB
        clawServoL = hardwareMap.get(CRServo.class, "claw_L"); // servo 2 - EXPANSION HUB

        closeClaw = false;
        openClaw = false;

//        clawServoR.getController().pwmEnable();
//        clawServoL.getController().pwmEnable();

        positionOfRotation = 0.5;   // Position Rotation servor to initial position
        clawRotationServo.setPosition(positionOfRotation);
        rotateToFront = false;
        rotateToBack = false;

        myGamePad2 = secondGamePad;
        clawTelemetry = telemetry;
    }

    // RotateClaw()
    // - Function to rotate (not open/close) the claw
    // - Servo is operating in standard mode (rotation limit 300 deg)
    public void RotateClaw() {

        if (rotateToFront) {
            positionOfRotation += INCREMENT ;

            // Keep stepping up until we hit the max value
            if (positionOfRotation >= MAX_POS ) {
                positionOfRotation = MAX_POS;
            }
        }
        else if (rotateToBack){
            positionOfRotation -= INCREMENT ;

            // Keep stepping down until we hit the min value
            if (positionOfRotation <= MIN_POS ) {
                positionOfRotation = MIN_POS;
            }
        }

        if (rotateToBack | rotateToFront ) {
            clawRotationServo.setPosition(positionOfRotation);
            rotateToFront = false;
            rotateToBack = false;
        }
    }

    // clawOpenClose()
    //   - The servos are operating in CONTINUOUS rotation mode. CVServo class is used.
    //   - Gamepad-1 controls are checked in the main program loop, and sets
    //     the 'closeClaw' and 'openClaw' variable values accordingly
    //   - If Y key is pressed, both servos operate to CLOSE the claw.
    //   - If A key is pressed, both servos operate to OPEN the claw.

    public void clawOpenClose() throws InterruptedException {                   // uses GAMEPAD-1 X and Y buttons

        if (closeClaw) {                // GAMEPAD-1 Y
            clawServoR.setDirection(CRServo.Direction.REVERSE);
            clawServoL.setDirection(CRServo.Direction.FORWARD);

//            clawServoR.getController().pwmEnable();
            clawServoR.setPower(0.3);
            clawServoL.setPower(0.3);

        }
        if (openClaw) {                 // GAMEPAD-1 X
            clawServoR.setDirection(CRServo.Direction.FORWARD);
            clawServoL.setDirection(CRServo.Direction.REVERSE);

//            clawServoR.getController().pwmEnable();
            clawServoR.setPower(0.3);
            clawServoL.setPower(0.3);

        }

        // Reset claw operation every time
        if (openClaw | closeClaw) {
            closeClaw = false;
            openClaw = false;
        }
        else
        {
            // stop the servos
            clawServoR.setPower(0.0);
            clawServoL.setPower(0.0);
        }
    }
} // end of class
