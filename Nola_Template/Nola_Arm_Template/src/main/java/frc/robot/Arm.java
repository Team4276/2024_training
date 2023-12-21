package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm {
    private static CANSparkMax motor;  
    private static Joystick joystick;
    
    
public static void init(){
    motor = new CANSparkMax(17, MotorType.kBrushless);
    joystick = new Joystick(0);
}

public static void update(){
    motor.set(joystick.getY() /4);
}
}
