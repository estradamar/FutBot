package org.firstinspires.ftc.teamcode;

/**
 * Máquina de estados finitos (FSM) del comportamiento autónomo de FutBot.
 *
 * Evaluación en orden estricto de prioridad (P1 = máxima):
 *
 *   P1  FAILSAFE_PENALTI    — línea blanca (color sensors) o portería propia muy cercana
 *   P2  ATAQUE_DISPARO      — pelota controlada → alinear y disparar a portería rival
 *   P3  ORBITA_INTERCEPCION — pelota visible → aproximar con anti-autogol orbital
 *   P4  BUSQUEDA_ACTIVA     — sin pelota → girar (con debounce de 200 ms)
 *
 * Uso típico (una vez por ciclo del bucle principal):
 *   vision.actualizarDatos();
 *   FutBotFSM.Estado estado = fsm.tick(hw, vision);
 */
public class FutBotFSM {

    // =========================================================================
    // Enum de estados
    // =========================================================================

    public enum Estado {
        FAILSAFE_PENALTI,
        ATAQUE_DISPARO,
        ORBITA_INTERCEPCION,
        BUSQUEDA_ACTIVA
    }

    // =========================================================================
    // Constantes calibrables — ajustar en cancha tras pruebas en campo
    // =========================================================================

    /** Ancho (px) del bbox de la portería PROPIA que indica "estamos dentro del área de penalti". */
    public static final int UMBRAL_ANCHO_PENALTI_PX = 120;

    /** Error angular máximo (px) para considerar un objeto "centrado" en la imagen. */
    public static final int UMBRAL_CENTRADO_PX = 25;

    /**
     * Error angular de la portería PROPIA (px) a partir del cual se activa la maniobra orbital.
     * Si la portería propia está más centrada que este umbral, el acercamiento directo produciría autogol.
     */
    public static final int UMBRAL_AUTOGOL_PX = 50;

    /** Ventana de debounce (ms): si la pelota desaparece menos de este tiempo, se mantiene el último comando. */
    public static final long DEBOUNCE_PELOTA_MS = 200;

    // --- Potencias de motor — rango [-1.0, 1.0] ---

    /** Ganancia del controlador proporcional (P) para el seguimiento angular de la pelota. */
    public static final double KP = 0.005;

    /** Potencia base de avance al aproximarse a la pelota en P3. */
    public static final double PODER_AVANCE = 0.55;

    /** Potencia de avance durante el disparo (P2 caso A: portería centrada). */
    public static final double PODER_ATAQUE = 0.70;

    /** Potencia de giro en eje: búsqueda activa y rotación de alineación de portería. */
    public static final double PODER_GIRO = 0.40;

    /** Motor exterior del arco orbital (la rueda que describe el arco más amplio). */
    public static final double PODER_ORBITA_RAPIDO = 0.60;

    /** Motor interior del arco orbital (la rueda que describe el arco más cerrado). */
    public static final double PODER_ORBITA_LENTO = 0.15;

    /** Potencia de retroceso/avance durante evasión de línea blanca o área de penalti. */
    public static final double PODER_EVASION = 0.55;

    // =========================================================================
    // Estado interno
    // =========================================================================

    private final int idRival;   // portería donde queremos anotar
    private final int idPropia;  // portería que defendemos

    private Estado estadoActual = Estado.BUSQUEDA_ACTIVA;

    // Debounce de pelota: evita tirones por 1-2 frames sin detección
    private long   tiempoUltimaVezVistaPelota = -1; // -1 = nunca detectada
    private double ultimoPoderIzq             = 0;
    private double ultimoPoderDer             = 0;

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * @param idPorteriaRival ID de la portería en la que queremos anotar.
     * @param idPorteriaPropia ID de la portería que debemos defender.
     */
    public FutBotFSM(int idPorteriaRival, int idPorteriaPropia) {
        this.idRival  = idPorteriaRival;
        this.idPropia = idPorteriaPropia;
    }

    // =========================================================================
    // Ciclo principal
    // =========================================================================

    /**
     * Evalúa los estados en orden de prioridad y aplica los comandos de motor.
     * Llamar UNA vez por ciclo del bucle, DESPUÉS de vision.actualizarDatos().
     *
     * @return Estado activo en este ciclo (para telemetría).
     */
    public Estado tick(FutBotHardware hw, FutBotVision vision) {

        // =====================================================================
        // P1 — FAILSAFE_PENALTI
        // Verificación de seguridad absoluta: línea blanca + área de penalti visual.
        // Si se activa, descarta todo lo demás y reacciona de inmediato.
        // =====================================================================

        if (hw.detectaBlancoNorte()) {
            // Sensor norte sobre la línea → retroceder
            estadoActual = Estado.FAILSAFE_PENALTI;
            aplicarPoder(hw, -PODER_EVASION, -PODER_EVASION);
            return estadoActual;
        }

        if (hw.detectaBlancoSur()) {
            // Sensor trasero sobre la línea → avanzar
            estadoActual = Estado.FAILSAFE_PENALTI;
            aplicarPoder(hw, PODER_EVASION, PODER_EVASION);
            return estadoActual;
        }

        if (vision.getAnchoPorteriaPorId(idPropia) >= UMBRAL_ANCHO_PENALTI_PX) {
            // Portería propia muy grande en cámara = estamos dentro del área de penalti
            estadoActual = Estado.FAILSAFE_PENALTI;
            aplicarPoder(hw, -PODER_EVASION, -PODER_EVASION);
            return estadoActual;
        }

        // =====================================================================
        // P2 — ATAQUE_DISPARO
        // El sensor de posesión detecta naranja: tenemos la pelota en posesión.
        // La cámara ya no interviene en esta decisión.
        // =====================================================================

        if (hw.detectaNaranjaPelota()) {
            estadoActual = Estado.ATAQUE_DISPARO;
            registrarPelotaVista();

            if (vision.hayPorteriaPorId(idRival)) {
                double errorPorteria = vision.getErrorAngularPorteriaPorId(idRival);

                if (Math.abs(errorPorteria) <= UMBRAL_CENTRADO_PX) {
                    // Caso A: portería rival centrada → potencia máxima + activar kicker
                    aplicarPoder(hw, PODER_ATAQUE, PODER_ATAQUE);
                    hw.kick();
                } else {
                    // Caso B: portería rival desviada → rotar sobre el eje para centrarla.
                    // error > 0 (portería a la derecha) → girar derecha (izq avanza, der retrocede).
                    double giro = PODER_GIRO * Math.signum(errorPorteria);
                    aplicarPoder(hw, giro, -giro);
                }
            } else {
                // Pelota controlada pero portería rival no visible → girar buscándola
                aplicarPoder(hw, PODER_GIRO, -PODER_GIRO);
            }

            return estadoActual;
        }

        // =====================================================================
        // P3 — ORBITA_INTERCEPCION
        // Cámara detecta pelota y el sensor de posesión no la tiene (P2 no activó).
        // Aproximación directa (control-P) o maniobra orbital si hay riesgo de autogol.
        // =====================================================================

        if (vision.hayPelota()) {
            estadoActual = Estado.ORBITA_INTERCEPCION;

            double errorPorteriaPropia = vision.getErrorAngularPorteriaPorId(idPropia);
            boolean riesgoAutoGol = vision.hayPorteriaPorId(idPropia)
                    && Math.abs(errorPorteriaPropia) < UMBRAL_AUTOGOL_PX;

            double poderIzq, poderDer;

            if (riesgoAutoGol) {
                // Portería propia casi centrada al frente: un empuje directo a la pelota
                // la dirigiría hacia nuestra propia portería.
                // Solución: arco amplio para rodear la pelota y posicionarse detrás.
                //
                // Si portería propia está a la DERECHA (error ≥ 0) → arco a la IZQUIERDA
                //   → motor der más rápido que izq
                // Si está a la IZQUIERDA (error < 0) → arco a la DERECHA
                //   → motor izq más rápido que der
                if (errorPorteriaPropia >= 0) {
                    poderIzq = PODER_ORBITA_LENTO;
                    poderDer = PODER_ORBITA_RAPIDO;
                } else {
                    poderIzq = PODER_ORBITA_RAPIDO;
                    poderDer = PODER_ORBITA_LENTO;
                }
            } else {
                // Sin riesgo de autogol: control proporcional directo sobre la pelota
                double error      = vision.getErrorAngularPelota(); // [-160, 160]
                double correccion = KP * error;
                poderIzq = PODER_AVANCE + correccion;
                poderDer = PODER_AVANCE - correccion;
            }

            registrarPelotaVista();
            aplicarPoder(hw, poderIzq, poderDer);
            return estadoActual;
        }

        // =====================================================================
        // P4 — BUSQUEDA_ACTIVA
        // Ni la cámara ve la pelota ni el sensor de posesión la detecta.
        // Debounce: si la cámara acaba de perderla (< DEBOUNCE_PELOTA_MS),
        // mantener el último comando para absorber frames espurios.
        // =====================================================================

        estadoActual = Estado.BUSQUEDA_ACTIVA;

        if (tiempoUltimaVezVistaPelota != -1) {
            long msSinPelota = System.currentTimeMillis() - tiempoUltimaVezVistaPelota;
            if (msSinPelota < DEBOUNCE_PELOTA_MS) {
                // Dentro de la ventana: congelar último comando de motor
                hw.setPoderMotores(ultimoPoderIzq, ultimoPoderDer);
                return estadoActual;
            }
        }

        // Fuera del debounce (o pelota nunca vista): girar en eje para escanear 360°
        aplicarPoder(hw, PODER_GIRO, -PODER_GIRO);
        return estadoActual;
    }

    // =========================================================================
    // Utilidades
    // =========================================================================

    /** @return Estado activo en el último ciclo. */
    public Estado getEstado() {
        return estadoActual;
    }

    /**
     * Aplica potencia a los motores y guarda los valores para el debounce de P4.
     * Usar este método en lugar de hw.setPoderMotores() dentro de la FSM.
     */
    private void aplicarPoder(FutBotHardware hw, double izq, double der) {
        ultimoPoderIzq = izq;
        ultimoPoderDer = der;
        hw.setPoderMotores(izq, der);
    }

    /** Actualiza el timestamp de la última detección de pelota. Llamar en P2 y P3. */
    private void registrarPelotaVista() {
        tiempoUltimaVezVistaPelota = System.currentTimeMillis();
    }
}
