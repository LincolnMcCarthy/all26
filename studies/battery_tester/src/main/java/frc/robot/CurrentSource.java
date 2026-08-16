package frc.robot;

import java.util.List;

import org.team100.lib.controller.r1.FeedbackR1;
import org.team100.lib.controller.r1.PIDFeedback;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.state.ModelR1;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Uses PWM controllers to produce the specified current.
 * 
 * The PWM controllers control duty-cycle, and the load is resistive, so the
 * output voltage is approximately the duty-cycle-scaled input voltage. The
 * current draw depends on the resistance of the load, which is not constant,
 * due to the large range of temperature.
 * 
 * For now, all the output devices are controlled together.
 * There are 5 controllers with 3 bulbs each, so 15 bulbs,
 * effectively all parallel.
 * 
 * Note: There's no mass in this system so there's no need to
 * control velocity.
 */
public class CurrentSource extends SubsystemBase {
    private static final boolean DEBUG = false;
    private final PowerDistribution pdh;
    private final List<VictorSP> controllers;
    private final FeedbackR1 feedback;
    private final LightBulb lightbulb;
    private final Battery battery;

    private final DoubleLogger m_log_power;
    private final DoubleLogger m_log_desired_power;
    private final DoubleLogger m_log_ff;
    private final DoubleLogger m_log_fb;
    private final DoubleLogger m_log_t;
    private final DoubleLogger m_log_dutycycle;
    private final DoubleLogger m_log_output_voltage;
    private final DoubleLogger m_log_output_current;
    private final DoubleLogger m_log_output_power;
    private final DoubleLogger m_log_battery_voltage;
    private final DoubleLogger m_log_sim_battery_voltage;

    // previously requested power (to avoid time travel)
    private double m_p;
    // previously commanded dutycycle (to avoid discretization error)
    private double m_dutycycle;

    public CurrentSource(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        pdh = new PowerDistribution(1, ModuleType.kRev);
        controllers = List.of(
                new VictorSP(0),
                new VictorSP(1),
                new VictorSP(2),
                new VictorSP(3),
                new VictorSP(4));
        feedback = new PIDFeedback(log, 0.0005, 0.00001, 0, false, 0.1, 1);
        lightbulb = new LightBulb();
        battery = new Battery();
        m_log_power = log.doubleLogger(Level.DEBUG, "power (W)");
        m_log_desired_power = log.doubleLogger(Level.DEBUG, "desired power (W)");
        m_log_ff = log.doubleLogger(Level.DEBUG, "ff");
        m_log_fb = log.doubleLogger(Level.DEBUG, "fb");
        m_log_t = log.doubleLogger(Level.DEBUG, "temperature (K)");
        m_log_dutycycle = log.doubleLogger(Level.DEBUG, "dutycycle");
        m_log_output_voltage = log.doubleLogger(Level.DEBUG, "output voltage (V)");
        m_log_output_current = log.doubleLogger(Level.DEBUG, "output current (A)");
        m_log_output_power = log.doubleLogger(Level.DEBUG, "output power (W)");
        m_log_battery_voltage = log.doubleLogger(Level.DEBUG, "battery voltage (V)");
        m_log_sim_battery_voltage = log.doubleLogger(Level.DEBUG, "sim battery voltage (V)");
    }

    /** Set bulb power (watts). */
    public void setPower(double p) {
        m_log_desired_power.log(() -> p);
        double ff = ff(p);
        m_log_ff.log(() -> ff);
        // feedback compares previous command to previous result
        // to avoid time travel
        double measurement = power();
        double setpoint = m_p;
        double fb = feedback.calculate(
                new ModelR1(measurement),
                new ModelR1(m_p));
        if (DEBUG)
            System.out.printf("measurement %f setpoint %f fb %f\n",
                    measurement, setpoint, fb);
        m_p = p;
        m_log_fb.log(() -> fb);
        m_dutycycle = MathUtil.clamp(ff + fb, 0, 1);
        controllers.stream().forEach(x -> x.set(m_dutycycle));
    }

    public void off() {
        setPower(0);
        feedback.reset();
        controllers.stream().forEach(VictorSP::stopMotor);
    }

    public double power() {
        if (RobotBase.isReal()) {
            return inputPower();
        }
        return outputPower();
    }

    private double inputPower() {
        return pdh.getTotalCurrent() * batteryVoltage();
    }

    private double outputPower() {
        return outputCurrent() * outputVoltage();
    }

    public double temperature() {
        return lightbulb.temperature(power());
    }

    /**
     * Feedforward duty cycle.
     * 
     * @param p desired output power, watts
     */
    private double ff(double p) {
        // Bulb voltage for the required power
        double v = lightbulb.VforP(p);
        // Use the modeled battery to avoid oscillation.
        // double vBatt = batteryVoltage();
        double vBatt = battery.VforP(p);
        // duty cycle given the battery voltage
        return MathUtil.clamp(v / vBatt, 0, 1);
    }

    /** Use duty cycle and light bulb model to find current. */
    private double outputCurrent() {
        double v = outputVoltage();
        return lightbulb.IforV(v);
    }

    /** Scale battery voltage by duty cycle. */
    private double outputVoltage() {
        return dutycycle() * batteryVoltage();
    }

    private double batteryVoltage() {
        return RobotController.getBatteryVoltage();
    }

    private double dutycycle() {
        // return controllers.get(0).get();
        return m_dutycycle;
    }

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            double p = powerForDutyCycle(lightbulb, battery, dutycycle());
            double v = battery.VforP(p);
            // System.out.printf("p %f v %f\n", p, v);
            m_log_sim_battery_voltage.log(() -> v);
            RoboRioSim.setVInVoltage(v);
        }
        m_log_power.log(this::power);
        m_log_t.log(this::temperature);
        m_log_dutycycle.log(this::dutycycle);
        m_log_output_voltage.log(this::outputVoltage);
        m_log_output_current.log(this::outputCurrent);
        m_log_output_power.log(this::outputPower);
        m_log_battery_voltage.log(this::batteryVoltage);
    }

    // static for testing
    // TODO: move to another class
    public static double batteryVoltageForDutycycle(
            LightBulb l, Battery b, double d) {
        double inputV = Battery.R;
        for (int j = 0; j < 100; ++j) {
            double outputV = d * inputV;
            double outputI = l.IforV(outputV);
            double p = outputV * outputI;
            double inputI = p / inputV;
            double v0 = inputV;
            inputV = b.VforI(inputI);
            // System.out.println(inputV);
            if (Math.abs(inputV - v0) < 0.001)
                return inputV;
        }
        System.out.println("WARNING CONVERGENCE");
        return 0;
    }

    public static double powerForDutyCycle(
            LightBulb l, Battery b, double d) {
        double inputV = batteryVoltageForDutycycle(l, b, d);
        double outputV = d * inputV;
        double outputI = l.IforV(outputV);
        double p = outputV * outputI;
        return p;
    }

}
