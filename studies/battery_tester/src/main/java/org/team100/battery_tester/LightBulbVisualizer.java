package org.team100.battery_tester;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LightBulbVisualizer {
    private final DoubleSupplier m_temperature;
    private final MechanismLigament2d m_filament;

    /**
     * @param temperature kelvin
     */
    public LightBulbVisualizer(DoubleSupplier temperature) {
        m_temperature = temperature;
        Mechanism2d m2d = new Mechanism2d(100, 100);
        m_filament = new MechanismLigament2d(
                "filament", 100, 0, 200, new Color8Bit(Color.kBlack));
        m2d.getRoot("root", 0, 50).append(m_filament);
        SmartDashboard.putData("lightbulb", m2d);
    }

    public void periodic() {
        m_filament.setColor(Zubetto.color(m_temperature.getAsDouble()));
    }

}