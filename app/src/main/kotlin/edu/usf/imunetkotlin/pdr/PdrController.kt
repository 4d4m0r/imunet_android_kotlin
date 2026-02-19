package edu.usf.imunetkotlin.pdr

interface PdrController {
    /** Inicializa sensores, modelo e estado interno. */
    fun initialize(): Boolean

    /** Liga ou desliga o modo PDR. */
    fun setEnable(enable: Boolean)

    /** Inicia ou para o rastreio em tempo real. */
    fun setStart(isStarted: Boolean)

    /** Reinicia todo o pipeline de forma forçada. */
    fun reset()

    /** Retorna posição relativa atual e passos estimados. */
    fun getTrajectory(): TrajectoryData

    /** Retorna a distância acumulada. */
    fun getDistance(): Float

    /** Retorna o tempo total desde o início do rastreio. */
    fun getDuration(): Long

    /** Define intervalo de atualização (200 a 500 ms). */
    fun setInterval(millis: Int): Boolean

    /** Define callback para receber snapshots de estado. */
    fun setListener(listener: PdrListener?)

    /** Libera sensores, modelo e thread de trabalho. */
    fun release()
}

fun interface PdrListener {
    /** Notifica estado e resultado mais recente do PDR. */
    fun onUpdate(snapshot: PdrSnapshot)
}

data class PdrSnapshot(
    val status: String,
    val enabled: Boolean,
    val started: Boolean,
    val sampleCount: Int,
    val lastDx: Float,
    val lastDy: Float,
    val distanceMeters: Float,
    val durationMillis: Long,
    val trajectory: TrajectoryData
)
