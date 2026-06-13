package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Mapeo y control centralizado del hardware de FutBot.
 * Todos los OpModes deben instanciar esta clase y llamar a init() antes de usarla.
 */
public class FutBotHardware {

    // --- Motores ---
    public DcMotor motorIzq = null;
    public DcMotor motorDer = null;

    // --- Sensores de color (REV Color Sensor V3 para detección de línea blanca) ---
    public NormalizedColorSensor sensorNorte  = null; // apunta al frente del robot
    public NormalizedColorSensor sensorSur    = null; // apunta hacia atrás
    public NormalizedColorSensor sensorEste   = null; // apunta al lado derecho
    public NormalizedColorSensor sensorOeste  = null; // apunta al lado izquierdo

    // --- Sensor táctil (botón pull-pin de arranque) ---
    public TouchSensor botonArranque = null;

    // Umbral de luminosidad para considerar que el sensor ve la línea blanca.
    // Ajustar tras calibración en campo; rango normalizado: 0.0 - 1.0.
    private static final float UMBRAL_BLANCO = 0.75f;

    // -------------------------------------------------------------------------
    // Inicialización
    // -------------------------------------------------------------------------

    /**
     * Mapea todos los dispositivos del HardwareMap y aplica la configuración inicial.
     * Debe llamarse desde runOpMode() antes de waitForStart().
     */
    public void init(HardwareMap hwMap) {

        // --- Mapeo de motores ---
        motorIzq = hwMap.get(DcMotor.class, "motorIzq");
        motorDer = hwMap.get(DcMotor.class, "motorDer");

        // El motor izquierdo está montado en espejo: se invierte para que
        // potencias positivas en ambos motores produzcan avance hacia adelante.
        motorIzq.setDirection(DcMotor.Direction.REVERSE);
        motorDer.setDirection(DcMotor.Direction.FORWARD);

        // Freno activo al cortar la energía para evitar deslizamiento.
        motorIzq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Modo sin encoder: control directo por potencia.
        motorIzq.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Asegurar que el robot no se mueva durante la inicialización.
        setPoderMotores(0, 0);

        // --- Mapeo de sensores de color (REV Color Sensor V3) ---
        sensorNorte = hwMap.get(NormalizedColorSensor.class, "sensorNorte");
        sensorSur   = hwMap.get(NormalizedColorSensor.class, "sensorSur");
        sensorEste  = hwMap.get(NormalizedColorSensor.class, "sensorEste");
        sensorOeste = hwMap.get(NormalizedColorSensor.class, "sensorOeste");

        // Configuración de ganancia (optimizado para V3)
        // Valores altos (ej. 15-30) ayudan a detectar la línea en condiciones de poca luz.
        sensorNorte.setGain(15);
        sensorSur.setGain(15);
        sensorEste.setGain(15);
        sensorOeste.setGain(15);

        // Activa el LED de los sensores si el hardware lo soporta.
        enableSensorLed(sensorNorte, true);
        enableSensorLed(sensorSur, true);
        enableSensorLed(sensorEste, true);
        enableSensorLed(sensorOeste, true);

        // --- Mapeo del botón de arranque ---
        botonArranque = hwMap.get(TouchSensor.class, "botonArranque");
    }

    /** Habilita o deshabilita el LED del sensor de color si es compatible. */
    private void enableSensorLed(NormalizedColorSensor sensor, boolean enable) {
        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight) sensor).enableLight(enable);
        }
    }

    // -------------------------------------------------------------------------
    // Control de motores
    // -------------------------------------------------------------------------

    /**
     * Aplica potencia a ambos motores. Valores esperados en [-1.0, 1.0].
     * Positivo = avance, negativo = retroceso.
     */
    public void setPoderMotores(double izq, double der) {
        motorIzq.setPower(izq);
        motorDer.setPower(der);
    }

    /** Detiene ambos motores. */
    public void detener() {
        setPoderMotores(0, 0);
    }

    // -------------------------------------------------------------------------
    // Detección de línea blanca (por sensor)
    // El canal alfa (alpha) del NormalizedColorSensor refleja la luminosidad total (0.0 a 1.0).
    // -------------------------------------------------------------------------

    /** @return true si el sensor frontal detecta la línea blanca de la cancha. */
    public boolean detectaBlancoNorte() {
        return sensorNorte.getNormalizedColors().alpha >= UMBRAL_BLANCO;
    }

    /** @return true si el sensor trasero detecta la línea blanca de la cancha. */
    public boolean detectaBlancoSur() {
        return sensorSur.getNormalizedColors().alpha >= UMBRAL_BLANCO;
    }

    /** @return true si el sensor derecho detecta la línea blanca de la cancha. */
    public boolean detectaBlancoEste() {
        return sensorEste.getNormalizedColors().alpha >= UMBRAL_BLANCO;
    }

    /** @return true si el sensor izquierdo detecta la línea blanca de la cancha. */
    public boolean detectaBlancoOeste() {
        return sensorOeste.getNormalizedColors().alpha >= UMBRAL_BLANCO;
    }

    /** @return true si el botón pull-pin de arranque está presionado (retenido). */
    public boolean botonPresionado() {
        return botonArranque.isPressed();
    }
}
