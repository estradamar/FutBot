package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FutBotFSM;
import org.firstinspires.ftc.teamcode.FutBotHardware;
import org.firstinspires.ftc.teamcode.FutBotVision;

/**
 * Base abstracta para los modos autónomos de FutBot.
 *
 * Contiene el ciclo principal, la inicialización de subsistemas y la telemetría.
 * Las subclases concretas solo declaran qué portería atacan y cuál defienden.
 *
 * Subclases disponibles:
 *   FutBotAutoAzul     — ataca portería azul,     defiende amarilla
 *   FutBotAutoAmarillo — ataca portería amarilla,  defiende azul
 */
public abstract class FutBotAuto extends LinearOpMode {

    // =========================================================================
    // Contrato de subclase
    // =========================================================================

    /** @return ID de la portería en la que queremos anotar. */
    protected abstract int getIdPorteriaObjetivo();

    /** @return ID de la portería que estamos defendiendo. */
    protected abstract int getIdPorteriaPropia();

    // =========================================================================
    // Subsistemas
    // =========================================================================

    protected final FutBotHardware hw     = new FutBotHardware();
    protected final FutBotVision   vision = new FutBotVision();
    protected FutBotFSM            fsm;

    // =========================================================================
    // Bucle principal
    // =========================================================================

    @Override
    public void runOpMode() {

        fsm = new FutBotFSM(getIdPorteriaObjetivo(), getIdPorteriaPropia());

        hw.init(hardwareMap);
        vision.init(hardwareMap);

        telemetry.addData("Objetivo", nombrePorteria(getIdPorteriaObjetivo()));
        telemetry.addData("Defiende", nombrePorteria(getIdPorteriaPropia()));
        telemetry.addData("Estado",   "Listo. Esperando START...");
        telemetry.update();

        /*
        // Arranque seguro con pull-pin físico (descomentar cuando esté disponible).
        while (opModeIsActive() && hw.botonPresionado()) {
            hw.detener();
            telemetry.addData("Arranque", "Suelta el boton pull-pin para iniciar");
            telemetry.update();
        }
        */

        waitForStart();

        // =====================================================================
        // BUCLE PRINCIPAL DEL PARTIDO
        // La FSM evalúa el estado y aplica los motores en cada ciclo.
        // =====================================================================
        while (opModeIsActive()) {

            vision.actualizarDatos();

            FutBotFSM.Estado estado = fsm.tick(hw, vision);

            imprimirTelemetria(estado);
            telemetry.update();
        }

        hw.detener();
    }

    // =========================================================================
    // Telemetría
    // =========================================================================

    private void imprimirTelemetria(FutBotFSM.Estado estado) {

        telemetry.addLine("─────────────────────────────────");
        telemetry.addData("ESTADO FSM", estado.name());
        telemetry.addLine("─────────────────────────────────");

        // Pelota
        if (vision.hayPelota()) {
            telemetry.addData("Pelota", "DETECTADA  err=%.0f px  area=%d px2",
                    vision.getErrorAngularPelota(), vision.getAreaPelota());
        } else {
            telemetry.addLine("Pelota:  no detectada");
        }

        // Portería rival (objetivo)
        int idObj = getIdPorteriaObjetivo();
        if (vision.hayPorteriaPorId(idObj)) {
            telemetry.addData("Rival [" + nombrePorteria(idObj) + "]",
                    "err=%.0f px  ancho=%d px",
                    vision.getErrorAngularPorteriaPorId(idObj),
                    vision.getAnchoPorteriaPorId(idObj));
        } else {
            telemetry.addData("Rival [" + nombrePorteria(idObj) + "]", "no visible");
        }

        // Portería propia
        int idProp = getIdPorteriaPropia();
        int anchoProp = vision.getAnchoPorteriaPorId(idProp);
        if (vision.hayPorteriaPorId(idProp)) {
            telemetry.addData("Propia [" + nombrePorteria(idProp) + "]",
                    "ancho=%d px  (limite=%d)",
                    anchoProp, FutBotFSM.UMBRAL_ANCHO_PENALTI_PX);
        } else {
            telemetry.addData("Propia [" + nombrePorteria(idProp) + "]", "no visible");
        }

        // Sensores de color
        telemetry.addData("Linea blanca",
                "Norte=%s  Sur=%s",
                hw.detectaBlancoNorte() ? "SI" : "--",
                hw.detectaBlancoSur()   ? "SI" : "--");
        telemetry.addData("Posesion (pelota)",
                hw.detectaNaranjaPelota() ? "PELOTA" : "--");

        telemetry.addLine("─────────────────────────────────");
    }

    private static String nombrePorteria(int id) {
        if (id == FutBotVision.ID_PORTERIA_AZUL)      return "AZUL";
        if (id == FutBotVision.ID_PORTERIA_AMARILLA)  return "AMARILLA";
        return "ID-" + id;
    }
}
