package frc.robot.subsystems.lift;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Lift extends SubsystemBase {
    
    //Motors
    private SparkMax m_liftMotorOne; //Motors to accuate the lift.
    private SparkMax m_liftMotorTwo;

    //Motor configs
    private SparkMaxConfig liftMotorOneConfig; //Configs for the motors moving the lift.
    private SparkMaxConfig liftMotorTwoConfig;


    //Lift states.
    enum liftStates {
        AT_INTAKE, //Lift is at the level where coral can be intaked via end effector.
        GOING_UP, //Lift is going up by input.
        GOING_DOWN, //Lift is going down by input
        AT_DIFFERENT_POSITION, //The lift was stopped at a position other than a defined position.
        LOW_CORAL, //Lift is at the low coral level.
        MEDIUM_CORAL, //Lift is at the medium coral level.
        HIGH_CORAL, //Lift is at the high coral level.
        BASE_CORAL, //Lift is at the base coral level.
        STOP_AT_DIFFERENT_POSITION //Get the lift ready to stop at a place not defined in the lift states.
    }



    //Constructor to intialize motors and other electronics.
    public Lift() {
        // Intialize motors
        m_liftMotorOne = new SparkMax(ClimbConstants.leftGrip_motorPort, MotorType.kBrushless); //Motors intialized for the lift.
        m_liftMotorTwo = new SparkMax(ClimbConstants.rightGrip_motorPort, MotorType.kBrushless);

        // Motor configs
        liftMotorOneConfig = new SparkMaxConfig(); //Configuartions for lift motors.
        liftMotorTwoConfig = new SparkMaxConfig();

        // Apply configurations and parameters to motor configs
        liftMotorOneConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake);
        liftMotorTwoConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake);

        // Set configs onto the motors.
        m_liftMotorOne.configure(
            liftMotorOneConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_liftMotorTwo.configure(
            liftMotorTwoConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }


    //Runs each cycle of the program to periodically check on the lift anf run other tasks.
    public void liftMain() {
        
    }

    //Controls the lift going up via user input.
    public void lift_runUp() {

    }

    //Controls the lift going down via user input.
    public void lift_runDown() {

    }


}
