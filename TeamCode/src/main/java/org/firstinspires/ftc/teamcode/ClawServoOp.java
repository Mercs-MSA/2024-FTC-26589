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
    static final double INCREMENT   = 0.007;     // amount to slew servo each CYCLE_MS cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // member variables - initialized in the constructor    
    double  positionR;
    double  positionL;
    boolean closeClaw;
    boolean openClaw;

    public Servo   clawServoR;
    public Servo   clawServoL;

    public Gamepad  myGamePad2;
    public Telemetry    clawTelemetry;

    // Constructor
    public ClawServoOp(@NonNull HardwareMap hardwareMap, Gamepad secondGamePad, Telemetry telemetry) {
        // claw initialization
        clawServoR = hardwareMap.get(Servo.class, "claw_R"); // servo 1
        clawServoL = hardwareMap.get(Servo.class, "claw_L"); // servo 2

        positionR = 0.5; // Start at MIN position
        positionL = 0.2; // Start at MAX position
        boolean closeClaw = false;
        boolean openClaw = false;

        myGamePad2 = secondGamePad;
        clawTelemetry = telemetry;

        clawServoR.setPosition(positionR);
        clawServoL.setPosition(positionL);

    }

    // OperateClaw() - Function to operate the claw.
    //   - It first checks if Y or A key is pressed on Gamepad-2.
    //   - If none of the two are found pressed, this function just returns without doing anything
    //   - If Y key is pressed, both servos operate to CLOSE the claw.
    //       - If MAX/MIN position reached, do nothing (just to protect the servo)
    //   - If A key is pressed, both servos operate to OPEN the claw.
    //       - If MAX/MIN position reached, do nothing (just to protect the servo)
    //

    public void OperateClaw() {

        if (myGamePad2 == null)
            return;

        // Operate the SERVO with Gamepad-2 commands
        if(myGamePad2.y) {      // OPEN the claw
            closeClaw = true;
            openClaw = false;
        }
        if(myGamePad2.x) {      // CLOSE the claw
            closeClaw = false;
            openClaw = true;
        }

        // slew the servo, according to the rampUp (direction) variable.
        if (closeClaw) {
            // Keep stepping up until we hit the max value.
            positionR += INCREMENT ;
                if (positionR >= MAX_POS ) {
                    positionR = MAX_POS;
                }
            positionL -= INCREMENT ;
            if (positionL <= MIN_POS ) {
                positionL = MIN_POS;
            }
        }
        if (openClaw) {
            // Keep stepping up until we hit the min value.
            positionR -= INCREMENT ;
                if (positionR <= MIN_POS ) {
                    positionR = MIN_POS;
                }
            positionL += INCREMENT ;
            if (positionL >= MAX_POS ) {
                positionL = MAX_POS;
            }
        }
        //telemetry.addData(">", "Press Stop to end test." );

        // Set the servo to the new position and pause;
        if (openClaw | closeClaw) {
            clawServoR.setPosition(positionR);
            clawServoL.setPosition(positionL);
            closeClaw = false;
            openClaw = false;

/*            // Display the current value
            clawTelemetry.addData("Servo  R  ", "%5.2f", clawServoR.getPosition());
            clawTelemetry.addData("Servo  L  ", "%5.2f", clawServoL.getPosition());
            clawTelemetry.update();
*/
        }

    } // end OperateClaw()

} // end of class
