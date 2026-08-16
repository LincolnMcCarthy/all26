package frc.robot;

import java.util.List;

import org.team100.lib.controller.r1.FeedbackR1;
import org.team100.lib.controller.r1.FullStateFeedback;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ModelR1;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.simulation.BatterySim;
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
    /**
     * key: current (measured or desired). The actual independent variable here is
     * temperature, which we can't really observe.
     * value: treating all controllers together, the effective resistance.
     */
    private final InterpolatingTreeMap<Double, Double> R;
    private final ProfileR1 profile;
    private final FeedbackR1 feedback;

    private ControlR1 iSetpoint;

    public CurrentSource(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        pdh = new PowerDistribution(1, ModuleType.kRev);
        controllers = List.of(
                new VictorSP(0),
                new VictorSP(1),
                new VictorSP(2),
                new VictorSP(3),
                new VictorSP(4));
        R = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
        // At zero output, resistance is *much* lower.
        // Note! This means we should vary the output
        // *slowly* !!
        R.put(0.0, 0.005);
        // At full output, each bulb draws 12.5 A at 12.0 V,
        // for 150W, so the resistance is about 0.96 ohm. Total
        // current is 12.5 * 15 = 187, and total resistance is
        // 12 / 187 = 0.064
        R.put(187.0, 0.064);
        // If output is even higher, resistance would also be higher.
        R.put(300.0, 0.1);
        // profile and feedback units are duty cycle, [0, 1]
        profile = new TrapezoidProfileR1(0.5, 1, 0.1);
        feedback = new FullStateFeedback(log, 1, 0, false, 0.1, 1);
        iSetpoint = new ControlR1();
    }

    public void setCurrent(double iGoal) {
        iSetpoint = profile.calculate(
                TimedRobot100.LOOP_PERIOD_S,
                iSetpoint,
                new ModelR1(iGoal));
        double ff = ff(iSetpoint.x());
        ModelR1 iMeasurement = new ModelR1(pdh.getTotalCurrent());
        double fb = feedback.calculate(iMeasurement, iSetpoint.model());
        double total = ff + fb;
        controllers.stream().forEach(x -> x.setVoltage(total));
    }

    public void off() {
        iSetpoint = new ControlR1();
        controllers.stream().forEach(VictorSP::stopMotor);
    }

    /** Feedforward duty cycle. */
    private double ff(double i) {
        i = MathUtil.clamp(i, 0, 300);
        // Use desired current to look up the resistance.
        double r = R.get(i);
        // Find the desired voltage using Ohm's law.
        double v = i * r;
        // Find the current actual battery voltage
        double vBatt = RobotController.getBatteryVoltage();
        // The ratio is the duty cycle.
        return MathUtil.clamp(v / vBatt, 0, 1);
    }

    private double simulatedAmps() {
        double dutyCycle = controllers.get(0).get();
        double v = dutyCycle * RobotController.getBatteryVoltage();
        // due to the strong temperature dependence of the light bulb,
        // the current draw needs to be solved iteratively.
        // NewtonsMethod1d solver = new NewtonsMethod1d(

        // );
        // return x * 30;
        return 0;
    }

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            double a = simulatedAmps();
            double v = BatterySim.calculateDefaultBatteryLoadedVoltage(a);
            RoboRioSim.setVInVoltage(v);
        }
    }

}
