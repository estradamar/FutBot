package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Wrapper de comunicación con la cámara HuskyLens.
 *
 * La HuskyLens opera en modo "Object Tracking" y devuelve bloques con:
 *   - ID del objeto reconocido
 *   - Coordenadas del centro (x, y) en píxeles
 *   - Dimensiones del bounding box (ancho, alto)
 *
 * Resolución de pantalla HuskyLens: 320 x 240 px → centro X = 160.
 */
public class FutBotVision {

    // -------------------------------------------------------------------------
    // Constantes de identificación de objetos (deben coincidir con el
    // entrenamiento almacenado en la HuskyLens).
    // -------------------------------------------------------------------------
    public static final int ID_PELOTA            = 2;
    public static final int ID_PORTERIA_AZUL     = 3;
    public static final int ID_PORTERIA_AMARILLA = 1;

    /** Columna central de la imagen (px). Se usa para calcular error angular. */
    public static final int CENTRO_X = 160;

    // -------------------------------------------------------------------------
    // Dispositivo I2C
    // -------------------------------------------------------------------------
    private HuskyLens huskyLens = null;

    // -------------------------------------------------------------------------
    // Estado interno actualizado por actualizarDatos()
    // -------------------------------------------------------------------------

    // Pelota
    private int xPelota     = -1; // -1 indica "no detectado"
    private int yPelota     = -1;
    private int anchoPelota = 0;
    private int altoPelota  = 0;
    private int idDetectado = -1; // ID del último objeto leído

    // Portería — campo "último visto" (mantiene compatibilidad con tests existentes)
    private int xPorteria           = -1;
    private int anchoPorteria       = 0;
    private int idPorteriaDetectada = -1;

    // Tracking por ID: permite consultar cada portería de forma independiente,
    // incluso si ambas son visibles en el mismo frame.
    private int xPorteriaAzul         = -1;
    private int anchoPorteriaAzul     = 0;
    private int xPorteriaAmarilla     = -1;
    private int anchoPorteriaAmarilla = 0;

    // -------------------------------------------------------------------------
    // Inicialización
    // -------------------------------------------------------------------------

    /**
     * Mapea la HuskyLens desde el HardwareMap usando el bus I2C configurado
     * en la Driver Station con el nombre "huskyLens".
     */
    public void init(HardwareMap hwMap) {
        huskyLens = hwMap.get(HuskyLens.class, "huskyLens");
        if (!huskyLens.knock()) {
            throw new RuntimeException("HuskyLens no responde — verificar cable I2C y nombre del dispositivo en la DS");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
    }

    // -------------------------------------------------------------------------
    // Lectura de datos
    // -------------------------------------------------------------------------

    /**
     * Solicita y parsea el frame más reciente de la HuskyLens.
     * Debe llamarse una vez por ciclo del bucle principal, ANTES de consultar
     * cualquier método de lógica (hayPelota, getErrorAngular, etc.).
     */
    public void actualizarDatos() {
        // Resetear estado antes de cada lectura
        xPelota               = -1;
        yPelota               = -1;
        anchoPelota           = 0;
        altoPelota            = 0;
        xPorteria             = -1;
        anchoPorteria         = 0;
        idPorteriaDetectada   = -1;
        xPorteriaAzul         = -1;
        anchoPorteriaAzul     = 0;
        xPorteriaAmarilla     = -1;
        anchoPorteriaAmarilla = 0;

        HuskyLens.Block[] blocks = huskyLens.blocks();
        if (blocks == null) return; // cámara sin respuesta — estado ya reseteado arriba

        for (HuskyLens.Block block : blocks) {
            
            if (block.id == ID_PELOTA) {
                xPelota = block.x;
                yPelota = block.y;
                anchoPelota = block.width;
                altoPelota = block.height;
                idDetectado = block.id;
            } else if (block.id == ID_PORTERIA_AZUL) {
                xPorteriaAzul     = block.x;
                anchoPorteriaAzul = block.width;
                // backward-compat: actualizar "último visto" también
                xPorteria           = block.x;
                anchoPorteria       = block.width;
                idPorteriaDetectada = block.id;
            } else if (block.id == ID_PORTERIA_AMARILLA) {
                xPorteriaAmarilla     = block.x;
                anchoPorteriaAmarilla = block.width;
                xPorteria             = block.x;
                anchoPorteria         = block.width;
                idPorteriaDetectada   = block.id;
            }
        }
    }

    // -------------------------------------------------------------------------
    // Métodos de consulta lógica
    // -------------------------------------------------------------------------

    /**
     * @return true si en el último frame se detectó la pelota (ID_PELOTA).
     */
    public boolean hayPelota() {
        return xPelota != -1;
    }

    /**
     * @param id ID de la portería a buscar (ID_PORTERIA_AZUL o ID_PORTERIA_AMARILLA).
     * @return true si en el último frame se detectó la portería con ese ID.
     */
    public boolean hayPorteria(int id) {
        return idPorteriaDetectada == id && xPorteria != -1;
    }

    /** @return Coordenada Y del centro de la pelota (0=arriba/lejos, 240=abajo/cerca); -1 si no detectada. */
    public int getYPelota() {
        return yPelota;
    }

    /**
     * Error angular de la pelota respecto al centro de la imagen.
     * Positivo → pelota a la derecha del centro (robot debe girar a la derecha).
     * Negativo → pelota a la izquierda (robot debe girar a la izquierda).
     *
     * @return error en píxeles; 0.0 si no hay pelota detectada.
     */
    public double getErrorAngularPelota() {
        if (!hayPelota()) return 0.0;
        return xPelota - CENTRO_X; // rango aprox. [-160, 160]
    }

    /**
     * Error angular de la portería respecto al centro de la imagen.
     *
     * @return error en píxeles; 0.0 si no hay portería detectada.
     */
    public double getErrorAngularPorteria() {
        if (xPorteria == -1) return 0.0;
        return xPorteria - CENTRO_X;
    }

    /**
     * Área aproximada del bounding box de la pelota (ancho × alto).
     * Usada como proxy de distancia: área grande → pelota cerca.
     *
     * @return área en px²; 0 si no hay pelota.
     */
    public int getAreaPelota() {
        if (!hayPelota()) return 0;
        return anchoPelota * altoPelota;
    }

    /**
     * Ancho del bounding box de la portería detectada.
     * Proxy de distancia a la portería.
     *
     * @return ancho en px; 0 si no hay portería.
     */
    public int getAnchoPorteria() {
        return anchoPorteria;
    }

    // -------------------------------------------------------------------------
    // Consulta por ID — tracking simultáneo de ambas porterías
    // Usados por FutBotFSM para razonar sobre portería propia y rival por separado.
    // -------------------------------------------------------------------------

    /**
     * @param id ID de la portería (ID_PORTERIA_AZUL o ID_PORTERIA_AMARILLA).
     * @return true si esa portería fue detectada en el último frame.
     */
    public boolean hayPorteriaPorId(int id) {
        if (id == ID_PORTERIA_AZUL)     return xPorteriaAzul     != -1;
        if (id == ID_PORTERIA_AMARILLA) return xPorteriaAmarilla != -1;
        return false;
    }

    /**
     * Error angular de una portería específica respecto al centro de la imagen.
     * Positivo → portería a la derecha del centro.
     *
     * @return error en px; 0.0 si esa portería no está visible.
     */
    public double getErrorAngularPorteriaPorId(int id) {
        if (id == ID_PORTERIA_AZUL     && xPorteriaAzul     != -1) return xPorteriaAzul     - CENTRO_X;
        if (id == ID_PORTERIA_AMARILLA && xPorteriaAmarilla != -1) return xPorteriaAmarilla - CENTRO_X;
        return 0.0;
    }

    /**
     * Ancho del bounding box de una portería específica. Proxy de distancia.
     *
     * @return ancho en px; 0 si esa portería no está visible.
     */
    public int getAnchoPorteriaPorId(int id) {
        if (id == ID_PORTERIA_AZUL)     return anchoPorteriaAzul;
        if (id == ID_PORTERIA_AMARILLA) return anchoPorteriaAmarilla;
        return 0;
    }

}
