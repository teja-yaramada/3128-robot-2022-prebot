package frc.team3128.common.hardware.motor;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.hal.SimDouble;

import frc.team3128.Robot;
import frc.team3128.common.NAR_EMotor;
import frc.team3128.Constants.ConversionConstants;

import net.thefletcher.revrobotics.CANSparkMax;
import net.thefletcher.revrobotics.SparkMaxRelativeEncoder;
import net.thefletcher.revrobotics.enums.MotorType;

public class NAR_CANSparkMax extends CANSparkMax implements NAR_EMotor {

	private double prevValue = 0;
	private SparkMaxRelativeEncoder encoder;
	private SimDeviceSim encoderSim;
	private SimDouble encoderPos;
	private SimDouble encoderVel;

	/**
	 * 
	 * @param deviceNumber device id
	 * @param type         0 for brushed motor, 1 for brushless motor
	 */
	public NAR_CANSparkMax(int deviceNumber, MotorType type) {
		super(deviceNumber, type);

		encoder = (SparkMaxRelativeEncoder)getEncoder();
		encoder.setPositionConversionFactor(ConversionConstants.SPARK_ENCODER_RESOLUTION); // convert rotations to encoder ticks - TEST

		if(Robot.isSimulation()){
			encoderSim = new SimDeviceSim("CANSparkMax[" + this.getDeviceId() + "] - RelativeEncoder");
			encoderPos = encoderSim.getDouble("Position");
			encoderVel = encoderSim.getDouble("Velocity");
		}

		// enableVoltageCompensation(true);
		// configVoltageCompSaturation(12, 10);
	}

	@Override
	public void set(double outputValue) {
		if (outputValue != prevValue) {
			super.set(outputValue);
			prevValue = outputValue;
		}
	}

	public double getSetpoint() {
		return prevValue;
	}

	@Override
	public double getMotorOutputVoltage() {
		return getAppliedOutput();
	}

	@Override
	public double getSelectedSensorPosition() {
		if(encoderSim != null)
			return encoderPos.get();	
		else
			return encoder.getPosition();
	}

	@Override
	public double getSelectedSensorVelocity() {
		if(encoderSim != null)
			return encoderVel.get();	
		else
			return encoder.getVelocity();
	}

	@Override
	public void setEncoderPosition(double pos) {
		if(encoderSim != null)
			encoderPos.set(pos);	
		else
			encoder.setPosition(pos);
	}

	@Override
	public void setSimPosition(double pos) {
		encoderPos.set(pos);
	}

	@Override
	public void setSimVelocity(double vel) {
		encoderVel.set(vel);
	}

	@Override
	public void follow(NAR_EMotor motor) {
		if(!(motor instanceof CANSparkMax)) {
			throw new RuntimeException("bad follow");
		}
		super.follow((CANSparkMax)motor);
	}
}
