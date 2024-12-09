//
// Class to handle all operations related to the INTAKE claw

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Auto_L_ClawServoOp_orig_claw {

    // Member variables
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    static final int REPEAT_LOOP    = 10;       // repeat loop to open/close

    // member variables - initialized in the constructor
    double positionOfRotation;
    double  positionR;
    double  positionL;
    boolean rotateToFront;
    boolean rotateToBack;

    public Servo   clawServoR;
    public Servo   clawServoL;
    public Servo   clawRotationServo;

    public Gamepad  myGamePad2;
    public Telemetry    clawTelemetry;

    // Constructor
    public Auto_L_ClawServoOp_orig_claw(@NonNull HardwareMap hardwareMap, Gamepad secondGamePad, Telemetry telemetry) {
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


    public void RotateClaw(double desiredPosition) throws InterruptedException {

        double currentPosition = clawRotationServo.getPosition();
        clawTelemetry.addData("currentPosition", "%4.2f", currentPosition);
        clawTelemetry.update();

        clawRotationServo.setPosition(desiredPosition);

        do {
            currentPosition = clawRotationServo.getPosition();
            clawTelemetry.addData("currentPosition", "%4.2f", currentPosition);
            clawTelemetry.update();
        } while(currentPosition < desiredPosition);

    }

    public void openClaw() throws InterruptedException {

        int repeat = REPEAT_LOOP;
        do {
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

            clawServoR.setPosition(positionR);
            clawServoL.setPosition(positionL);

        } while (repeat-- > 0);
    }

    public void closeClaw() throws InterruptedException {

        int repeat = REPEAT_LOOP;
        do {
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
            clawTelemetry.addData("Claw Status", "CLOSED");
            clawTelemetry.update();

            clawServoR.setPosition(positionR);
            clawServoL.setPosition(positionL);

        } while (repeat-- > 0);
    }
} // end of class
