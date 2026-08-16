package frc.robot;

import java.util.List;

import org.team100.lib.controller.r1.FeedbackR1;
import org.team100.lib.controller.r1.FullStateFeedback;
import org.team100.lib.logging.LoggerFactory;
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

    private final PowerDistribution pdh;
    private final List<VictorSP> controllers;
    private final FeedbackR1 feedback;
    private final LightBulb lightbulb;
    private final Battery battery;

    public CurrentSource(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        pdh = new PowerDistribution(1, ModuleType.kRev);
        controllers = List.of(
                new VictorSP(0),
                new VictorSP(1),
                new VictorSP(2),
                new VictorSP(3),
                new VictorSP(4));
        feedback = new FullStateFeedback(log, 1, 0, false, 0.1, 1);
        lightbulb = new LightBulb();
        battery = new Battery();
    }

    /** Set bulb power (watts). */
    public void setPower(double p) {
        double ff = ff(p);
        double fb = feedback.calculate(
                new ModelR1(pdh.getTotalCurrent() * RobotController.getBatteryVoltage()),
                new ModelR1(p));
        double dutyCycle = ff + fb;
        controllers.stream().forEach(x -> x.set(dutyCycle));
    }

    public void off() {
        controllers.stream().forEach(VictorSP::stopMotor);
    }

    /**
     * Feedforward duty cycle.
     * 
     * @param p desired output power, watts
     */
    private double ff(double p) {
        // bulb voltage for the required power
        double v = lightbulb.VforP(p);
        // Get the current voltage (which might be simulated, see below).
        double vBatt = RobotController.getBatteryVoltage();
        // duty cycle given the battery voltage
        return MathUtil.clamp(v / vBatt, 0, 1);
    }

    private double simulatedAmps() {
        double dutyCycle = controllers.get(0).get();
        double v = dutyCycle * RobotController.getBatteryVoltage();
        return lightbulb.IforV(v);
    }

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            double a = simulatedAmps();
            double v = battery.VforI(a);
            RoboRioSim.setVInVoltage(v);
        }
    }

}
