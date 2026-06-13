package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FutBotHardware;
import org.firstinspires.ftc.teamcode.FutBotVision;

/**
 * OpMode autónomo de FutBot con objetivo de anotar en la PORTERÍA AMARILLA.
 *
 * Idéntico a FutBotAutoAzul salvo por ID_PORTERIA_OBJETIVO.
 *
 * Máquina de estados priorizada:
 *   P0 - Supervivencia : evadir la línea blanca
 *   P1 - Ataque        : perseguir la pelota con control proporcional
 *   P2 - Anotación     : empujar la pelota hacia la portería amarilla
 *   P3 - Radar         : girar para buscar la pelota
 *   P4 - Reposicionamiento : avanzar hacia la portería para cambiar perspectiva
 */
@Autonomous(name = "FutBot - Portería AMARILLA", group = "FutBot")
public class FutBotAutoAmarillo extends LinearOpMode {

    // -------------------------------------------------------------------------
    // Constantes de ajuste de comportamiento
    // -------------------------------------------------------------------------

    /** Ganancia proporcional del controlador P para seguimiento angular. */
    private static final double KP = 0.005;

    /** Potencia base de avance al perseguir la pelota. */
    private static final double PODER_AVANCE = 0.55;

    /** Potencia base de avance al empujar hacia la portería. */
    private static final double PODER_ATAQUE = 0.60;

    /** Potencia de giro al buscar la pelota (Radar). */
    private static final double PODER_GIRO_RADAR = 0.40;

    /** Potencia de reposicionamiento (avance hacia portería para cambiar ángulo). */
    private static final double PODER_REPOSICION = 0.45;

    /**
     * Umbral de área del bounding box de la pelota (px²) que indica
     * que la pelota está suficientemente cerca como para intentar anotar.
     */
    private static final int UMBRAL_AREA_PELOTA_CONTROLADA = 3000;

    /** Tiempo máximo en estado Radar antes de activar Reposicionamiento (ms). */
    private static final long TIEMPO_MAX_RADAR_MS = 3000;

    /** Duración del avance de reposicionamiento (ms). */
    private static final long DURACION_REPOSICION_MS = 2000;

    /** Duración de la maniobra de evasión de línea blanca (ms). */
    private static final long TIEMPO_EVASION_MS = 1000;

    /** Portería objetivo de este OpMode. */
    private static final int ID_PORTERIA_OBJETIVO = FutBotVision.ID_PORTERIA_AMARILLA;

    // -------------------------------------------------------------------------
    // Subsistemas
    // -------------------------------------------------------------------------
    private FutBotHardware hw     = new FutBotHardware();
    private FutBotVision   vision = new FutBotVision();

    // -------------------------------------------------------------------------
    // Temporizadores
    // -------------------------------------------------------------------------
    private ElapsedTime tiempoRadar = new ElapsedTime();

    // -------------------------------------------------------------------------
    // Punto de entrada del OpMode
    // -------------------------------------------------------------------------

    @Override
    public void runOpMode() {

        // --- Inicialización de subsistemas ---
        hw.init(hardwareMap);
        vision.init(hardwareMap);

        telemetry.addData("Estado", "Inicializado. Esperando START...");
        telemetry.update();

        // Esperar a que el árbitro presione START en la Driver Station.
        waitForStart();

        // --- Arranque Seguro (Strict Rule) ---
        // El botón físico pull-pin debe estar retenido (presionado) durante el
        // posicionamiento del robot. El partido real empieza cuando se suelta.
        while (opModeIsActive() && hw.botonPresionado()) {
            hw.detener();
            telemetry.addData("Arranque", "Suelta el botón pull-pin para iniciar");
            telemetry.update();
        }

        // Resetear cronómetro del Radar al arrancar.
        tiempoRadar.reset();

        // =====================================================================
        // BUCLE PRINCIPAL DEL PARTIDO
        // =====================================================================
        while (opModeIsActive()) {

            // Leer cámara una vez por ciclo.
            vision.actualizarDatos();

            // =================================================================
            // PRIORIDAD 0 — SUPERVIVENCIA: Evasión de línea blanca
            // La penalización por tocar la línea es 1 minuto fuera de campo,
            // por lo que esta prioridad nunca puede ser ignorada.
            // =================================================================

            if (hw.detectaBlancoFrenteIzq() || hw.detectaBlancoFrenteDer()) {
                // Arco frontal detecta línea → retroceder
                telemetry.addData("Evasion", "FRENTE - Retrocediendo");
                telemetry.update();
                hw.setPoderMotores(-PODER_AVANCE, -PODER_AVANCE);
                sleep(TIEMPO_EVASION_MS);
                hw.detener();
                tiempoRadar.reset();
                continue;

            } else if (hw.detectaBlancoSur()) {
                // Arco trasero detecta línea → avanzar
                telemetry.addData("Evasion", "SUR - Avanzando");
                telemetry.update();
                hw.setPoderMotores(PODER_AVANCE, PODER_AVANCE);
                sleep(TIEMPO_EVASION_MS);
                hw.detener();
                tiempoRadar.reset();
                continue;
            }

            // =================================================================
            // PRIORIDAD 2 — ANOTACIÓN: Pelota cerca, buscar y empujar portería
            // =================================================================

            else if (vision.hayPelota() && vision.getAreaPelota() > UMBRAL_AREA_PELOTA_CONTROLADA) {

                if (vision.hayPorteria(ID_PORTERIA_OBJETIVO)) {
                    // Portería visible: corregir ángulo y empujar con la pelota
                    double errorPorteria = vision.getErrorAngularPorteria();
                    double correccion    = KP * errorPorteria;
                    double motorIzq      = PODER_ATAQUE + correccion;
                    double motorDer      = PODER_ATAQUE - correccion;

                    hw.setPoderMotores(motorIzq, motorDer);

                    telemetry.addData("Estado", "ANOTANDO hacia porteria amarilla");
                    telemetry.addData("Error porteria (px)", errorPorteria);

                } else {
                    // Pelota controlada pero portería no visible: girar buscándola.
                    hw.setPoderMotores(PODER_GIRO_RADAR, -PODER_GIRO_RADAR);
                    telemetry.addData("Estado", "BUSCANDO porteria amarilla (pelota controlada)");
                }

            }

            // =================================================================
            // PRIORIDAD 1 — ATAQUE: Pelota visible, perseguirla con control P
            // =================================================================

            else if (vision.hayPelota()) {

                double error      = vision.getErrorAngularPelota(); // [-160, 160]
                double correccion = KP * error;

                double poderIzq = PODER_AVANCE + correccion;
                double poderDer = PODER_AVANCE - correccion;

                hw.setPoderMotores(poderIzq, poderDer);

                tiempoRadar.reset();

                telemetry.addData("Estado", "PERSIGUIENDO pelota");
                telemetry.addData("Error angular (px)", error);
                telemetry.addData("Area pelota (px2)", vision.getAreaPelota());

            }

            // =================================================================
            // PRIORIDAD 4 — REPOSICIONAMIENTO: Radar agotado → cambiar perspectiva
            // =================================================================

            else if (tiempoRadar.milliseconds() > TIEMPO_MAX_RADAR_MS) {

                if (vision.hayPorteria(ID_PORTERIA_OBJETIVO)) {
                    telemetry.addData("Estado", "REPOSICIONANDO hacia porteria amarilla");
                    telemetry.update();

                    hw.setPoderMotores(PODER_REPOSICION, PODER_REPOSICION);
                    sleep(DURACION_REPOSICION_MS);
                    hw.detener();

                } else {
                    hw.setPoderMotores(PODER_GIRO_RADAR, -PODER_GIRO_RADAR);
                    telemetry.addData("Estado", "REPOSICIONANDO - Buscando porteria");
                }

                tiempoRadar.reset();

            }

            // =================================================================
            // PRIORIDAD 3 — RADAR: Girar sobre el eje buscando la pelota
            // =================================================================

            else {
                hw.setPoderMotores(PODER_GIRO_RADAR, -PODER_GIRO_RADAR);
                telemetry.addData("Estado", "RADAR - Buscando pelota");
                telemetry.addData("Tiempo radar (ms)", tiempoRadar.milliseconds());
            }

            telemetry.update();
        }

        // Asegurar parada completa al finalizar el OpMode.
        hw.detener();
    }
}
