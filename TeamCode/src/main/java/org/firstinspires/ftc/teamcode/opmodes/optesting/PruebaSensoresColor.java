package org.firstinspires.ftc.teamcode.opmodes.optesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.FutBotHardware;
import org.firstinspires.ftc.teamcode.tests.ColorSensorDiagnostico;

/**
 * OpMode de prueba para verificar los 3 sensores de color del sistema de evasión.
 *
 * Cómo usar:
 *   1. Seleccionar este OpMode en la Driver Station.
 *   2. Presionar INIT y luego PLAY.
 *   3. Pasar cada sensor sobre la línea blanca de la cancha uno a la vez.
 *   4. Verificar en la DS que el sensor correcto muestra "DETECTADO" y la acción esperada.
 *   5. Presionar STOP cuando termine la prueba.
 *
 * Los motores NO se mueven durante esta prueba.
 */
@TeleOp(name = "TEST - Sensores de Color", group = "FutBot Testing")
public class PruebaSensoresColor extends LinearOpMode {

    private static final float UMBRAL_BLANCO = 0.75f;

    @Override
    public void runOpMode() {

        // Inicializar hardware completo (mismo código que en producción).
        FutBotHardware hw = new FutBotHardware();
        hw.init(hardwareMap);

        // Sensores de línea blanca (usan canal alpha).
        ColorSensorDiagnostico[] sensores = {
            new ColorSensorDiagnostico(
                hw.sensorNorte,
                "Norte (Línea Frente)",
                "RETROCEDER 1 seg",
                UMBRAL_BLANCO
            ),
            new ColorSensorDiagnostico(
                hw.sensorSur,
                "Sur (Línea Trasera)",
                "AVANZAR 1 seg",
                UMBRAL_BLANCO
            )
        };

        telemetry.addLine("=== PRUEBA SENSORES COLOR ===");
        telemetry.addLine("Listo. Presiona PLAY para comenzar.");
        telemetry.update();

        waitForStart();

        // =====================================================================
        // BUCLE DE PRUEBA — solo lectura, motores siempre a 0
        // =====================================================================
        while (opModeIsActive()) {

            hw.detener(); // garantía: motores apagados en todo momento

            telemetry.addLine("─────────────────────────────────");
            telemetry.addLine("  PRUEBA DE SENSORES DE COLOR");
            telemetry.addLine("  Pasa cada sensor sobre la linea blanca");
            telemetry.addLine("─────────────────────────────────");
            telemetry.addLine("");

            boolean algunaLinea = false;
            boolean posesion    = false;

            // --- Sensores de línea blanca ---
            for (ColorSensorDiagnostico s : sensores) {

                float alpha       = s.getAlpha();
                boolean detectado = s.detectaBlanco();

                String indicador = detectado ? ">>> BLANCO DETECTADO <<<" : "sin linea";
                telemetry.addData(s.getNombre(), "alpha=%.3f  |  %s", alpha, indicador);

                if (detectado) {
                    telemetry.addData("  Accion en produccion", s.getAccionImplicada());
                    algunaLinea = true;
                }
            }

            // --- Sensor de pelota (posesión) — una sola lectura I2C para display y detección ---
            NormalizedRGBA cp = hw.sensorPelota.getNormalizedColors();
            posesion           = cp.red >= 0.35f && cp.blue <= 0.12f; // mismos umbrales que FutBotHardware
            String indPelota   = posesion ? ">>> NARANJA DETECTADA <<<" : "sin pelota";
            telemetry.addData("Pelota (Posesion)", "r=%.3f b=%.3f  |  %s", cp.red, cp.blue, indPelota);
            if (posesion) {
                telemetry.addData("  Accion en produccion", "POSESION CONFIRMADA");
            }

            telemetry.addLine("");
            telemetry.addLine("─────────────────────────────────");

            // Resumen de estado global
            if (algunaLinea) {
                telemetry.addLine("  ESTADO: LINEA DETECTADA — se ejecutaria evasion");
            } else if (posesion) {
                telemetry.addLine("  ESTADO: PELOTA EN POSESION — se activaria P2");
            } else {
                telemetry.addLine("  ESTADO: campo libre, sin detecciones");
            }

            telemetry.addLine("─────────────────────────────────");
            telemetry.addData("Umbral configurado", "%.2f", UMBRAL_BLANCO);
            telemetry.update();
        }
    }
}
