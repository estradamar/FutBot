package org.firstinspires.ftc.teamcode.opmodes.optesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FutBotHardware;

/**
 * Prueba: motores avanzan a velocidad fija y se detienen en seco al detectar blanco.
 *
 * Cómo usar:
 *   1. Colocar el robot en el campo, lejos de la línea blanca.
 *   2. Seleccionar este OpMode en la Driver Station.
 *   3. Presionar INIT y luego PLAY.
 *   4. El robot avanza a 0.15 de potencia.
 *   5. Al cruzar cualquier sensor sobre la línea blanca, los motores paran en seco.
 *   6. Presionar STOP para terminar.
 */
@TeleOp(name = "TEST - Motores con Linea", group = "FutBot Testing")
public class PruebaMotoresConLinea extends LinearOpMode {

    private static final double VELOCIDAD_AVANCE = 0.15;

    @Override
    public void runOpMode() {

        FutBotHardware hw = new FutBotHardware();
        hw.init(hardwareMap);

        telemetry.addLine("=== PRUEBA MOTORES CON LINEA ===");
        telemetry.addLine("El robot avanzara a 0.15 y frenara al detectar blanco.");
        telemetry.addLine("Listo. Presiona PLAY para comenzar.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean blancoNorte     = hw.detectaBlancoNorte();
            boolean blancoSur       = hw.detectaBlancoSur();
            boolean lineaDetectada  = blancoNorte || blancoSur;

            if (lineaDetectada) {
                hw.detener();
            } else {
                hw.setPoderMotores(VELOCIDAD_AVANCE, VELOCIDAD_AVANCE);
            }

            // --- Telemetría ---
            telemetry.addLine("─────────────────────────────────");
            telemetry.addLine("  PRUEBA MOTORES CON LINEA BLANCA");
            telemetry.addLine("─────────────────────────────────");
            telemetry.addData("Velocidad configurada", "%.2f", VELOCIDAD_AVANCE);
            telemetry.addLine("");
            telemetry.addData("Norte (frente)", blancoNorte ? "BLANCO DETECTADO" : "libre");
            telemetry.addData("Sur (trasero)",  blancoSur   ? "BLANCO DETECTADO" : "libre");
            telemetry.addLine("");

            if (lineaDetectada) {
                telemetry.addLine("  >>> FRENADO EN SECO — LINEA DETECTADA <<<");
            } else {
                telemetry.addLine("  AVANZANDO a 0.15");
            }

            telemetry.addLine("─────────────────────────────────");
            telemetry.update();
        }

        hw.detener();
    }
}
