package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

/**
 * Wrapper de diagnóstico para un RevColorSensorV3.
 *
 * Encapsula un sensor físico junto con su nombre en el robot y la acción
 * que dispararía en el sistema de evasión real. Se usa exclusivamente en
 * OpModes de prueba para mostrar telemetría estructurada en la DS.
 */
public class ColorSensorDiagnostico {

    private final RevColorSensorV3 sensor;
    private final String nombre;
    private final String accionSiDetecta;
    private final float umbralBlanco;

    /**
     * @param sensor         Sensor físico ya mapeado desde el HardwareMap.
     * @param nombre         Nombre descriptivo que aparecerá en la DS (ej. "FrenteIzq").
     * @param accionSiDetecta Texto de la acción que ejecutaría el robot al detectar blanco.
     * @param umbralBlanco   Valor alpha (0.0–1.0) a partir del cual se considera blanco.
     */
    public ColorSensorDiagnostico(RevColorSensorV3 sensor,
                                   String nombre,
                                   String accionSiDetecta,
                                   float umbralBlanco) {
        this.sensor          = sensor;
        this.nombre          = nombre;
        this.accionSiDetecta = accionSiDetecta;
        this.umbralBlanco    = umbralBlanco;
    }

    /** @return Lectura actual del canal alfa normalizado (0.0 a 1.0). */
    public float getAlpha() {
        return sensor.getNormalizedColors().alpha;
    }

    /** @return Lectura RGBA completa en una sola llamada I2C (usar para telemetría detallada). */
    public NormalizedRGBA getRawColors() {
        return sensor.getNormalizedColors();
    }

    /** @return true si la lectura actual supera el umbral de blanco. */
    public boolean detectaBlanco() {
        return getAlpha() >= umbralBlanco;
    }

    /** @return Nombre del sensor tal como aparecerá en telemetría. */
    public String getNombre() {
        return nombre;
    }

    /**
     * @return Texto de la acción que el robot ejecutaría en producción si este
     *         sensor disparara la evasión (ej. "RETROCEDER 1s").
     */
    public String getAccionImplicada() {
        return detectaBlanco() ? accionSiDetecta : "---";
    }

    /** @return Umbral configurado para este sensor. */
    public float getUmbral() {
        return umbralBlanco;
    }
}
