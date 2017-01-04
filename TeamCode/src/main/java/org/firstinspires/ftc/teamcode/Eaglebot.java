package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * The robot is the Eaglebot
 *
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Eaglebot
{
    /* Public OpMode members. */
    public DcMotor frontRight = null;
    public DcMotor frontLeft = null;
    public DcMotor backRight = null;
    public DcMotor backLeft = null;
    public Servo  leftArm = null;
    public Servo rightArm = null;
    public Servo ballPusher = null;
    public LightSensor lightSensorBeacon = null;
    public LightSensor lightSensorFloor = null;
    public double FORWARD_SPEED = 0.6;
    public double TURN_SPEED = 0.5;
    public double threshold = 1.94;
    public double TOP_SPEED = 1.0;
    public double ARM_SPEED = 0.25;
    public double launch_speed = 0.25;
   // public OpticalDistanceSensor odsSensor = null;
    public DcMotor armMotor = null;
    public DcMotor launcher = null;



    // public static final double MID_SERVO       =  0.5 ;
    // public static final double ARM_UP_POWER    =  0.45 ;
    // public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Eaglebot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeft  = hwMap.dcMotor.get("nw_motor");
        backLeft = hwMap.dcMotor.get("sw_motor");
        frontRight = hwMap.dcMotor.get("ne_motor");
        backRight = hwMap.dcMotor.get("se_motor");
        lightSensorBeacon = hwMap.lightSensor.get("light_sensor_beacon");
        lightSensorFloor = hwMap.lightSensor.get ("light_sensor_floor");
        // odsSensor = hwMap.opticalDistanceSensor.get("optical_distance");
        leftArm = hwMap.servo.get("left_arm");
        rightArm = hwMap.servo.get("right_arm");
        rightArm.setDirection(Servo.Direction.REVERSE);
        ballPusher = hwMap.servo.get("paddle");
        armMotor = hwMap.dcMotor.get("arm_motor");
        launcher = hwMap.dcMotor.get("launch_motor");



        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        launcher.setPower(0);
        armMotor.setPower(0);

        leftArm.setPosition(0);
        rightArm.setPosition(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Define and initialize ALL installed servos.
        // leftClaw = hwMap.servo.get("left_hand");
        // rightClaw = hwMap.servo.get("right_hand");
        // leftClaw.setPosition(MID_SERVO);
        // rightClaw.setPosition(MID_SERVO);
    }
    void forward(double speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);
    }
    void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }
    void turn (double speed) {
        frontLeft.setPower(-speed);
        backRight.setPower(speed);
    }

    void leftTurn (double speed) {
        frontRight.setPower(speed);
        backLeft.setPower(-speed);
    }
    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

