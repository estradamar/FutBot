package org.firstinspires.ftc.teamcode.opmodes.optesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

        // Crear un diagnóstico por sensor, con nombre y acción de evasión real.
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
            ),
            new ColorSensorDiagnostico(
                hw.sensorPelota,
                "Pelota (Posesion)",
                "POSESION CONFIRMADA",
                UMBRAL_BLANCO // Usando blanco como referencia para prueba genérica
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

            boolean alguno = false;

            for (ColorSensorDiagnostico s : sensores) {

                float alpha        = s.getAlpha();
                boolean detectado  = s.detectaBlanco();

                // Línea de estado: nombre + valor + indicador visual
                String indicador = detectado ? ">>> BLANCO DETECTADO <<<" : "sin linea";
                telemetry.addData(s.getNombre(), "alpha=%.3f  |  %s", alpha, indicador);

                if (detectado) {
                    telemetry.addData("  Accion en produccion", s.getAccionImplicada());
                    alguno = true;
                }
            }

            telemetry.addLine("");
            telemetry.addLine("─────────────────────────────────");

            // Resumen de estado global
            if (alguno) {
                telemetry.addLine("  ESTADO: LINEA DETECTADA — se ejecutaria evasion");
            } else {
                telemetry.addLine("  ESTADO: campo libre, sin lineas detectadas");
            }

            telemetry.addLine("─────────────────────────────────");
            telemetry.addData("Umbral configurado", "%.2f", UMBRAL_BLANCO);
            telemetry.update();
        }
    }
}
