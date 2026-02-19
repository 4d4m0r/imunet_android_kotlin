package edu.usf.imunetkotlin.pdr

import android.content.Context
import android.content.res.AssetFileDescriptor
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import edu.usf.imunetkotlin.Quaternion
import org.tensorflow.lite.Interpreter
import org.tensorflow.lite.flex.FlexDelegate
import java.io.FileInputStream
import java.nio.MappedByteBuffer
import java.nio.channels.FileChannel
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.floor
import kotlin.math.hypot

class PdrEngine(
    context: Context,
    private val modelAssetName: String = "IMUNet.tflite"
) : PdrController, SensorEventListener {

    companion object {
        private const val WINDOW_SIZE = 200
        private const val CHANNELS = 6
        private const val INFER_EVERY_N_SAMPLES = 5
        private const val MIN_INTERVAL_MS = 200
        private const val MAX_INTERVAL_MS = 500
        private const val DEFAULT_INTERVAL_MS = 200
        private const val DEFAULT_STEP_LENGTH_M = 0.75f
    }

    private val appContext = context.applicationContext
    private val sensorManager = appContext.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val worker = Executors.newSingleThreadExecutor()
    private val mainHandler = Handler(Looper.getMainLooper())
    private val inferenceInFlight = AtomicBoolean(false)
    private val stateLock = Any()

    private var listener: PdrListener? = null

    private var accSensor: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private var gyroSensor: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
    private var rotSensor: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR)

    private var interpreter: Interpreter? = null
    private var flexDelegate: FlexDelegate? = null

    private var enabled = false
    private var started = false
    private var intervalMs = DEFAULT_INTERVAL_MS
    private var status = "PDR desligado"

    private var startBaseMs = 0L
    private var accumulatedDurationMs = 0L
    private var lastInferenceWallClockMs = 0L
    private var lastPublishWallClockMs = 0L

    private val window = Array(CHANNELS) { FloatArray(WINDOW_SIZE) }
    private var sampleCount = 0
    private var inferenceTicker = 0

    private var latestAcc = FloatArray(3)
    private var latestGyro = FloatArray(3)
    private var latestRotation: Quaternion? = null
    private var hasAcc = false
    private var hasGyro = false

    private var initRotor: Quaternion? = null
    private var lastGyroTimestampNs = 0L
    private var dtMeanSec = 0f
    private var dtCount = 0

    private var posX = 0f
    private var posY = 0f
    private var stepsCount = 0
    private var distanceMeters = 0f
    private var lastDx = 0f
    private var lastDy = 0f

    /** Reseta estado, valida sensores e carrega o modelo. */
    override fun initialize(): Boolean {
        val ready = synchronized(stateLock) {
            clearTrackingStateLocked()
            status = "Inicializando"
            val hasSensors = accSensor != null && gyroSensor != null && rotSensor != null
            if (!hasSensors) {
                status = "Sensores indisponiveis (acelerometro/giroscopio/rotacao)"
                false
            } else {
                loadModelLocked()
            }
        }
        publishSnapshot(force = true)
        return ready
    }

    /** Ativa/desativa o modo PDR e para captura quando desligado. */
    override fun setEnable(enable: Boolean) {
        synchronized(stateLock) {
            if (enabled == enable) return
            enabled = enable
            if (!enable) {
                internalStopLocked()
                status = "PDR desabilitado"
            } else {
                if (interpreter == null) {
                    val initialized = loadModelLocked()
                    status = if (initialized) "PDR habilitado" else status
                } else {
                    status = "PDR habilitado"
                }
            }
        }
        publishSnapshot(force = true)
    }

    /** Inicia ou para aquisição e inferência em tempo real. */
    override fun setStart(isStarted: Boolean) {
        synchronized(stateLock) {
            if (!enabled && isStarted) {
                status = "Ative PDR antes de iniciar"
                publishSnapshot(force = true)
                return
            }

            if (!isStarted) {
                internalStopLocked()
                status = "PDR parado"
                publishSnapshot(force = true)
                return
            }

            if (started) return
            if (interpreter == null && !loadModelLocked()) {
                publishSnapshot(force = true)
                return
            }
            if (accSensor == null || gyroSensor == null || rotSensor == null) {
                status = "Sensores indisponiveis (acelerometro/giroscopio/rotacao)"
                publishSnapshot(force = true)
                return
            }

            clearTrackingStateLocked()
            started = true
            startBaseMs = SystemClock.elapsedRealtime()
            registerSensorsLocked()
            status = "PDR em execucao"
        }
        publishSnapshot(force = true)
    }

    /** Reinicia todo o sistema e recarrega o modelo. */
    override fun reset() {
        val shouldResume = synchronized(stateLock) { enabled && started }
        synchronized(stateLock) {
            internalStopLocked()
            closeModelLocked()
            clearTrackingStateLocked()
            status = "PDR resetado"
            loadModelLocked()
        }
        if (shouldResume) {
            setStart(true)
        } else {
            publishSnapshot(force = true)
        }
    }

    /** Retorna coordenadas relativas e passos atuais. */
    override fun getTrajectory(): TrajectoryData = synchronized(stateLock) {
        TrajectoryData(posX, posY, stepsCount)
    }

    /** Retorna a distância acumulada em metros. */
    override fun getDistance(): Float = synchronized(stateLock) { distanceMeters }

    /** Retorna tempo total de execução do rastreio. */
    override fun getDuration(): Long = synchronized(stateLock) {
        if (!started) return@synchronized accumulatedDurationMs
        accumulatedDurationMs + (SystemClock.elapsedRealtime() - startBaseMs)
    }

    /** Define intervalo de atualização permitido (200..500 ms). */
    override fun setInterval(millis: Int): Boolean {
        if (millis !in MIN_INTERVAL_MS..MAX_INTERVAL_MS) return false
        synchronized(stateLock) {
            intervalMs = millis
            status = "Intervalo configurado: ${intervalMs}ms"
        }
        publishSnapshot(force = true)
        return true
    }

    /** Registra callback para receber snapshots do engine. */
    override fun setListener(listener: PdrListener?) {
        synchronized(stateLock) {
            this.listener = listener
        }
        publishSnapshot(force = true)
    }

    /** Libera sensores, modelo e recursos de thread. */
    override fun release() {
        synchronized(stateLock) {
            internalStopLocked()
            closeModelLocked()
        }
        worker.shutdownNow()
    }

    /** Recebe eventos de sensores e despacha processamento por tipo. */
    override fun onSensorChanged(event: SensorEvent) {
        synchronized(stateLock) {
            if (!enabled || !started) return
            try {
                when (event.sensor.type) {
                    Sensor.TYPE_ACCELEROMETER -> {
                        latestAcc = event.values.copyOfRange(0, 3)
                        hasAcc = true
                    }

                    Sensor.TYPE_GAME_ROTATION_VECTOR -> {
                        val q = Quaternion.fromRotationVector(event.values)
                        latestRotation = q
                        if (initRotor == null) {
                            initRotor = q.conjugate().normalized()
                        }
                    }

                    Sensor.TYPE_GYROSCOPE -> {
                        latestGyro = event.values.copyOfRange(0, 3)
                        hasGyro = true
                        processGyroTickLocked(event.timestamp)
                    }
                }
            } catch (t: Throwable) {
                internalStopLocked()
                status = "Erro de sensor: ${t.javaClass.simpleName}: ${t.message}"
                publishSnapshot(force = true)
            }
        }
    }

    /** Não usado neste fluxo. */
    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    /** Processa tick do giroscópio: orienta sinais, atualiza janela e roda inferência. */
    private fun processGyroTickLocked(timestampNs: Long) {
        val rotation = latestRotation ?: return
        val base = initRotor ?: return
        if (!hasAcc || !hasGyro) return

        if (lastGyroTimestampNs == 0L) {
            lastGyroTimestampNs = timestampNs
            return
        }

        val dt = (timestampNs - lastGyroTimestampNs) / 1_000_000_000.0f
        lastGyroTimestampNs = timestampNs
        if (dt <= 0f || dt > 0.2f) return

        dtCount += 1
        dtMeanSec += (dt - dtMeanSec) / dtCount

        val ori = (base * rotation).normalized()
        val orientedGyro = ori.rotateVector(latestGyro)
        val orientedAcc = ori.rotateVector(latestAcc)
        appendWindowLocked(orientedGyro, orientedAcc)

        if (sampleCount < WINDOW_SIZE) {
            maybePublishSnapshotLocked(force = false)
            return
        }

        inferenceTicker += 1
        if (inferenceTicker % INFER_EVERY_N_SAMPLES != 0) return

        val nowMs = SystemClock.elapsedRealtime()
        if (nowMs - lastInferenceWallClockMs < intervalMs) return
        if (!inferenceInFlight.compareAndSet(false, true)) return

        lastInferenceWallClockMs = nowMs
        val input = snapshotInputLocked()
        val dtForInference = dtMeanSec

        worker.execute {
            try {
                val output = Array(1) { FloatArray(2) }
                interpreter?.run(input, output)

                val dx = output[0][0] * dtForInference
                val dy = output[0][1] * dtForInference

                synchronized(stateLock) {
                    posX += dx
                    posY += dy
                    lastDx = dx
                    lastDy = dy
                    distanceMeters += hypot(dx, dy)
                    stepsCount = floor(distanceMeters / DEFAULT_STEP_LENGTH_M).toInt()
                }
                publishSnapshot(force = false)
            } catch (t: Throwable) {
                synchronized(stateLock) {
                    internalStopLocked()
                    status = "Falha na inferencia: ${t.javaClass.simpleName}: ${t.message}"
                }
                publishSnapshot(force = true)
            } finally {
                inferenceInFlight.set(false)
            }
        }
    }

    /** Adiciona nova amostra na janela deslizante [6 x 200]. */
    private fun appendWindowLocked(gyro: FloatArray, acc: FloatArray) {
        if (sampleCount < WINDOW_SIZE) {
            window[0][sampleCount] = gyro[0]
            window[1][sampleCount] = gyro[1]
            window[2][sampleCount] = gyro[2]
            window[3][sampleCount] = acc[0]
            window[4][sampleCount] = acc[1]
            window[5][sampleCount] = acc[2]
            sampleCount += 1
            return
        }

        for (c in 0 until CHANNELS) {
            val arr = window[c]
            System.arraycopy(arr, 1, arr, 0, WINDOW_SIZE - 1)
        }
        window[0][WINDOW_SIZE - 1] = gyro[0]
        window[1][WINDOW_SIZE - 1] = gyro[1]
        window[2][WINDOW_SIZE - 1] = gyro[2]
        window[3][WINDOW_SIZE - 1] = acc[0]
        window[4][WINDOW_SIZE - 1] = acc[1]
        window[5][WINDOW_SIZE - 1] = acc[2]
        sampleCount += 1
    }

    /** Gera cópia do input no formato esperado pelo modelo [1][6][200]. */
    private fun snapshotInputLocked(): Array<Array<FloatArray>> {
        val input = Array(1) { Array(CHANNELS) { FloatArray(WINDOW_SIZE) } }
        for (c in 0 until CHANNELS) {
            System.arraycopy(window[c], 0, input[0][c], 0, WINDOW_SIZE)
        }
        return input
    }

    /** Registra listeners dos sensores com máxima taxa. */
    private fun registerSensorsLocked() {
        accSensor?.also { sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_FASTEST) }
        gyroSensor?.also { sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_FASTEST) }
        rotSensor?.also { sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_FASTEST) }
    }

    /** Para captura e congela a duração acumulada. */
    private fun internalStopLocked() {
        if (!started) return
        sensorManager.unregisterListener(this)
        accumulatedDurationMs = getDuration()
        started = false
        inferenceInFlight.set(false)
    }

    /** Limpa posição, janelas, contadores e estados temporários. */
    private fun clearTrackingStateLocked() {
        for (c in 0 until CHANNELS) {
            java.util.Arrays.fill(window[c], 0f)
        }
        sampleCount = 0
        inferenceTicker = 0
        hasAcc = false
        hasGyro = false
        latestRotation = null
        initRotor = null
        lastGyroTimestampNs = 0L
        dtMeanSec = 0f
        dtCount = 0
        posX = 0f
        posY = 0f
        stepsCount = 0
        distanceMeters = 0f
        lastDx = 0f
        lastDy = 0f
        startBaseMs = 0L
        accumulatedDurationMs = 0L
        lastInferenceWallClockMs = 0L
        lastPublishWallClockMs = 0L
    }

    /** Carrega modelo TFLite, com fallback sem FlexDelegate. */
    private fun loadModelLocked(): Boolean {
        closeModelLocked()

        try {
            flexDelegate = FlexDelegate()
            val options = Interpreter.Options().addDelegate(flexDelegate)
            interpreter = Interpreter(loadModelFile(modelAssetName), options)
            status = "Modelo pronto (FlexDelegate)"
            return true
        } catch (t: Throwable) {
            flexDelegate?.close()
            flexDelegate = null
        }

        return try {
            interpreter = Interpreter(loadModelFile(modelAssetName))
            status = "Modelo pronto (sem FlexDelegate)"
            true
        } catch (t: Throwable) {
            interpreter = null
            status = "Falha ao carregar modelo: ${t.javaClass.simpleName}: ${t.message}"
            false
        }
    }

    /** Fecha interpreter e delegate atuais. */
    private fun closeModelLocked() {
        interpreter?.close()
        interpreter = null
        flexDelegate?.close()
        flexDelegate = null
    }

    /** Faz memory-map do arquivo de modelo em assets. */
    private fun loadModelFile(fileName: String): MappedByteBuffer {
        val fd: AssetFileDescriptor = appContext.assets.openFd(fileName)
        fd.use {
            FileInputStream(it.fileDescriptor).channel.use { channel ->
                return channel.map(FileChannel.MapMode.READ_ONLY, it.startOffset, it.declaredLength)
            }
        }
    }

    /** Publica snapshot apenas quando respeitar o intervalo configurado. */
    private fun maybePublishSnapshotLocked(force: Boolean) {
        val nowMs = SystemClock.elapsedRealtime()
        if (!force && nowMs - lastPublishWallClockMs < intervalMs) return
        lastPublishWallClockMs = nowMs
        publishSnapshot(force = true)
    }

    /** Monta e envia snapshot para a UI no main thread. */
    private fun publishSnapshot(force: Boolean) {
        val snapshot = synchronized(stateLock) {
            if (!force) {
                val nowMs = SystemClock.elapsedRealtime()
                if (nowMs - lastPublishWallClockMs < intervalMs) return@synchronized null
                lastPublishWallClockMs = nowMs
            }

            PdrSnapshot(
                status = status,
                enabled = enabled,
                started = started,
                sampleCount = sampleCount.coerceAtMost(WINDOW_SIZE),
                lastDx = lastDx,
                lastDy = lastDy,
                distanceMeters = distanceMeters,
                durationMillis = getDuration(),
                trajectory = TrajectoryData(posX, posY, stepsCount)
            )
        }
        if (snapshot == null) return

        mainHandler.post {
            listener?.onUpdate(snapshot)
        }
    }
}
