package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Mapeo y control centralizado del hardware de FutBot.
 * Todos los OpModes deben instanciar esta clase y llamar a init() antes de usarla.
 */
public class FutBotHardware {

    // --- Motores ---
    public DcMotor motorIzq = null;
    public DcMotor motorDer = null;

    // --- Sensores de color ---
    //
    //          [Norte]         ← línea blanca norte
    //             |
    //          [Robot]──[Pelota]  ← detecta pelota naranja en posesión
    //             |
    //          [ Sur ]         ← línea blanca sur
    //
    public RevColorSensorV3 sensorSur    = null; // línea blanca — arco sur/trasero
    public RevColorSensorV3 sensorNorte  = null; // línea blanca — arco norte/frente
    public RevColorSensorV3 sensorPelota = null; // pelota naranja — posesión

    // --- Sensor táctil (botón pull-pin de arranque) ---
    // public TouchSensor botonArranque = null;

    // Ajustar tras calibración en campo; rango normalizado: 0.0 - 1.0.
    private static final float UMBRAL_BLANCO    = 0.75f; // línea blanca: canal alpha
    private static final float UMBRAL_NARANJA_R = 0.35f; // naranja: mínimo rojo
    private static final float UMBRAL_NARANJA_B = 0.12f; // naranja: máximo azul

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
        sensorSur      = hwMap.get(RevColorSensorV3.class, "sensorSur");
        sensorNorte    = hwMap.get(RevColorSensorV3.class, "sensorNorte");
        sensorPelota = hwMap.get(RevColorSensorV3.class, "sensorPelota");

        // Ganancia alta para detectar blanco (línea) y naranja (pelota) en campo.
        sensorSur.setGain(15);
        sensorNorte.setGain(15);
        sensorPelota.setGain(15);

        // LED encendido: ilumina el objeto y mejora la lectura de color.
        enableSensorLed(sensorSur, true);
        enableSensorLed(sensorNorte, true);
        enableSensorLed(sensorPelota, true);

        // --- Mapeo del botón de arranque ---
        // botonArranque = hwMap.get(TouchSensor.class, "botonArranque");
    }

    /** Habilita o deshabilita el LED del sensor de color. */
    private void enableSensorLed(RevColorSensorV3 sensor, boolean enable) {
        sensor.enableLed(enable);
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
    // Detección de línea blanca
    // Canal alfa normalizado (0.0 a 1.0); blanco supera típicamente 0.75.
    // -------------------------------------------------------------------------

    /** @return true si el sensor sur detecta la línea blanca. */
    public boolean detectaBlancoSur() {
        return sensorSur.getNormalizedColors().alpha >= UMBRAL_BLANCO;
    }

    /** @return true si el sensor norte detecta la línea blanca. */
    public boolean detectaBlancoNorte() {
        return sensorNorte.getNormalizedColors().alpha >= UMBRAL_BLANCO;
    }

    // -------------------------------------------------------------------------
    // Detección de pelota naranja — sensor de posesión/posesión
    // Comprueba canal rojo alto y azul bajo para aislar el color naranja.
    // -------------------------------------------------------------------------

    /** @return true si el sensor de posesión detecta la pelota naranja (posesión). */
    public boolean detectaNaranjaPelota() {
        float r = sensorPelota.getNormalizedColors().red;
        float b = sensorPelota.getNormalizedColors().blue;
        return r >= UMBRAL_NARANJA_R && b <= UMBRAL_NARANJA_B;
    }

    /** @return true si el botón pull-pin de arranque está presionado (retenido). */
    public boolean botonPresionado() {
        // return botonArranque.isPressed();
        return false;
    }

    // -------------------------------------------------------------------------
    // Mecanismo de disparo (kicker / driblador)
    // -------------------------------------------------------------------------

    // TODO: Mapear el actuador cuando el hardware esté definido.
    //   Servo:  private Servo kickerServo = null;
    //   Motor:  private DcMotor dribladorMotor = null;
    // Y en init():
    //   kickerServo = hwMap.get(Servo.class, "kickerServo");

    /**
     * Activa el mecanismo de disparo/driblador un ciclo.
     * Sin efecto hasta que se conecte y mapee el hardware físico.
     *
     * Implementación sugerida con servo:
     *   kickerServo.setPosition(1.0);
     *   sleep(150);
     *   kickerServo.setPosition(0.0);
     */
    public void kick() {
        // placeholder — activar cuando el hardware esté listo
    }
}
