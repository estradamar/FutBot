package org.firstinspires.ftc.teamcode.opmodes.optesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FutBotVision;

/**
 * OpMode de prueba para verificar la cámara HuskyLens.
 *
 * Cómo usar:
 *   1. Seleccionar este OpMode en la Driver Station.
 *   2. Presionar INIT y luego PLAY.
 *   3. Apuntar la cámara a la pelota, portería amarilla o portería azul.
 *   4. Verificar en la DS que el objeto correcto muestra "DETECTADO" con su posición.
 *   5. Presionar STOP cuando termine la prueba.
 *
 * Los motores NO se mueven durante esta prueba.
 */
@TeleOp(name = "TEST - Camara HuskyLens", group = "FutBot Testing")
public class PruebaCamara extends LinearOpMode {

    @Override
    public void runOpMode() {

        FutBotVision vision = new FutBotVision();
        vision.init(hardwareMap);

        telemetry.addLine("=== PRUEBA CAMARA HUSKYLENS ===");
        telemetry.addLine("Listo. Presiona PLAY para comenzar.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            vision.actualizarDatos();

            telemetry.addLine("─────────────────────────────────");
            telemetry.addLine("  PRUEBA DE CAMARA HUSKYLENS");
            telemetry.addLine("  Apunta la camara a un objeto");
            telemetry.addLine("─────────────────────────────────");
            telemetry.addLine("");

            // --- Pelota ---
            if (vision.hayPelota()) {
                telemetry.addLine("PELOTA:  >>> DETECTADA <<<");
                telemetry.addData("  Error angular", "%.1f px", vision.getErrorAngularPelota());
                telemetry.addData("  Area (distancia aprox)", "%d px2", vision.getAreaPelota());
            } else {
                telemetry.addLine("PELOTA:  no detectada");
            }

            telemetry.addLine("");

            // --- Portería amarilla ---
            if (vision.hayPorteria(FutBotVision.ID_PORTERIA_AMARILLA)) {
                telemetry.addLine("PORTERIA AMARILLA:  >>> DETECTADA <<<");
                telemetry.addData("  Error angular", "%.1f px", vision.getErrorAngularPorteria());
                telemetry.addData("  Ancho (distancia aprox)", "%d px", vision.getAnchoPorteria());
            } else {
                telemetry.addLine("PORTERIA AMARILLA:  no detectada");
            }

            telemetry.addLine("");

            // --- Portería azul ---
            if (vision.hayPorteria(FutBotVision.ID_PORTERIA_AZUL)) {
                telemetry.addLine("PORTERIA AZUL:  >>> DETECTADA <<<");
                telemetry.addData("  Error angular", "%.1f px", vision.getErrorAngularPorteria());
                telemetry.addData("  Ancho (distancia aprox)", "%d px", vision.getAnchoPorteria());
            } else {
                telemetry.addLine("PORTERIA AZUL:  no detectada");
            }

            telemetry.addLine("");
            telemetry.addLine("─────────────────────────────────");

            boolean hayAlgo = vision.hayPelota()
                    || vision.hayPorteria(FutBotVision.ID_PORTERIA_AMARILLA)
                    || vision.hayPorteria(FutBotVision.ID_PORTERIA_AZUL);

            if (hayAlgo) {
                telemetry.addLine("  ESTADO: OBJETO DETECTADO");
            } else {
                telemetry.addLine("  ESTADO: sin objetos en camara");
            }

            telemetry.addLine("─────────────────────────────────");
            telemetry.update();
        }
    }
}
