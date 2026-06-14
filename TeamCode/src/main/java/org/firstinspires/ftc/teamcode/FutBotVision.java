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
    public HuskyLens huskyLens = null;

    // -------------------------------------------------------------------------
    // Estado interno actualizado por actualizarDatos()
    // -------------------------------------------------------------------------

    // Pelota
    private int xPelota     = -1; // -1 indica "no detectado"
    private int yPelota     = -1;
    private int anchoPelota = 0;
    private int altoPelota  = 0;
    private int idDetectado = -1; // ID del último objeto leído

    // Portería
    private int xPorteria     = -1;
    private int anchoPorteria = 0;
    private int idPorteriaDetectada = -1;

    // -------------------------------------------------------------------------
    // Inicialización
    // -------------------------------------------------------------------------

    /**
     * Mapea la HuskyLens desde el HardwareMap usando el bus I2C configurado
     * en la Driver Station con el nombre "huskyLens".
     */
    public void init(HardwareMap hwMap) {
        huskyLens = hwMap.get(HuskyLens.class, "huskyLens");
        
        // Inicializa el dispositivo para empezar a comunicarse
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
        xPelota             = -1;
        yPelota             = -1;
        anchoPelota         = 0;
        altoPelota          = 0;
        xPorteria           = -1;
        anchoPorteria       = 0;
        idPorteriaDetectada = -1;

        HuskyLens.Block[] blocks = huskyLens.blocks();
        
        for (int i = 0; i < blocks.length; i++) {
            HuskyLens.Block block = blocks[i];
            
            if (block.id == ID_PELOTA) {
                xPelota = block.x;
                yPelota = block.y;
                anchoPelota = block.width;
                altoPelota = block.height;
                idDetectado = block.id;
            } else if (block.id == ID_PORTERIA_AZUL || block.id == ID_PORTERIA_AMARILLA) {
                xPorteria = block.x;
                anchoPorteria = block.width;
                idPorteriaDetectada = block.id;
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

}
