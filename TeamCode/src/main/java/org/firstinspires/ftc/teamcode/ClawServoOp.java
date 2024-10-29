//
// Class to handle all operations related to the INTAKE claw

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClawServoOp {

    // Member variables
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // member variables - initialized in the constructor    
    double positionOfRotation;
    double  positionR;
    double  positionL;
    boolean rotateToFront;
    boolean rotateToBack;
    boolean closeClaw;
    boolean openClaw;

    public Servo   clawServoR;
    public Servo   clawServoL;
    public Servo   clawRotationServo;

    public Gamepad  myGamePad2;
    public Telemetry    clawTelemetry;

    // Constructor
    public ClawServoOp(@NonNull HardwareMap hardwareMap, Gamepad secondGamePad, Telemetry telemetry) {
        // claw initialization
        clawRotationServo = hardwareMap.get(Servo.class, "claw_Rotation"); // servo 0
        clawServoR = hardwareMap.get(Servo.class, "claw_R"); // servo 1
        clawServoL = hardwareMap.get(Servo.class, "claw_L"); // servo 2

        clawServoL.resetDeviceConfigurationForOpMode();
        clawServoL.scaleRange(0.0,0.6);
        positionL = 0.6;
        clawServoL.setDirection(Servo.Direction.REVERSE);
        clawServoL.setPosition(positionL);

        clawServoR.resetDeviceConfigurationForOpMode();
        clawServoR.scaleRange(0.0,0.3);
        positionR = 0.8;
        clawServoR.setPosition(positionR);

        closeClaw = false;
        openClaw = false;

        positionOfRotation = 0.5;   // Position Rotation servor to initial position
        clawRotationServo.setPosition(positionOfRotation);
        rotateToFront = false;
        rotateToBack = false;


        myGamePad2 = secondGamePad;
        clawTelemetry = telemetry;

    }

    // OperateClaw() - Function to operate the claw.
    //   - It first checks if Y or A key is pressed on Gamepad-2.
    //   - If none of the two are found pressed, this function just returns without doing anything
    //   - If Y key is pressed, both servos operate to CLOSE the claw.
    //       - If MAX/MIN position reached, do nothing (just to protect the servo)
    //   - If A key is pressed, both servos operate to OPEN the claw.
    //       - If MAX/MIN position reached, do nothing (just to protect the servo)
    //


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

            // Keep stepping up until we hit the min value
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

    public void setOpenClaw() {
        positionR = MIN_POS;
        positionL = MAX_POS;
        clawServoR.setPosition(positionR);
        clawServoL.setPosition(positionL);

        clawTelemetry.addData("Claw Status", "Opened");
        clawTelemetry.update();
    }

    public void clawOpenClose() {                   // uses GAMEPAD-1 X and Y buttons

        if (closeClaw) {
            positionR += INCREMENT;
            // Keep stepping up until we hit the max value.
            if (positionR >= MAX_POS) {
                positionR = MAX_POS;
            }
            positionL += INCREMENT;
            // Keep stepping up until we hit the max value.
            if (positionL >= MAX_POS) {
                positionL = MAX_POS;
            }
        }
        if (openClaw) {
            positionL -= INCREMENT;
            // Keep stepping up until we hit the min value.
            if (positionL <= MIN_POS) {
                positionL = MIN_POS;
            }
            positionR -= INCREMENT;
            // Keep stepping up until we hit the min value.
            if (positionR <= MIN_POS) {
                positionR = MIN_POS;
            }
        }

        // Set the servo to the new position and pause;
        if (openClaw | closeClaw) {
            clawServoR.setPosition(positionR);
            clawServoL.setPosition(positionL);
            closeClaw = false;
            openClaw = false;
        }
        if (rotateToBack | rotateToFront) {
            clawRotationServo.setPosition(positionOfRotation);
            rotateToFront = false;
            rotateToBack = false;
        }
    }
} // end of class
