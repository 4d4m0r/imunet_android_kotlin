package edu.usf.imunetkotlin

import android.content.res.AssetFileDescriptor
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.widget.Button
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import com.github.mikephil.charting.charts.ScatterChart
import com.github.mikephil.charting.components.XAxis
import com.github.mikephil.charting.data.Entry
import com.github.mikephil.charting.data.ScatterData
import com.github.mikephil.charting.data.ScatterDataSet
import org.tensorflow.lite.Interpreter
import org.tensorflow.lite.flex.FlexDelegate
import java.io.FileInputStream
import java.nio.MappedByteBuffer
import java.nio.channels.FileChannel
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean

class MainActivity : AppCompatActivity(), SensorEventListener {

    companion object {
        private const val MODEL_FILE = "IMUNet.tflite"
        private const val WINDOW_SIZE = 200
        private const val CHANNELS = 6
        private const val INFER_EVERY_N_SAMPLES = 5
    }

    private lateinit var sensorManager: SensorManager
    private var gyroSensor: Sensor? = null
    private var accSensor: Sensor? = null
    private var rotSensor: Sensor? = null

    private var interpreter: Interpreter? = null
    private var flexDelegate: FlexDelegate? = null

    private lateinit var statusText: TextView
    private lateinit var sampleText: TextView
    private lateinit var outputText: TextView
    private lateinit var chart: ScatterChart

    private lateinit var trajectorySet: ScatterDataSet
    private lateinit var scatterData: ScatterData

    private val worker = Executors.newSingleThreadExecutor()
    private val ui = Handler(Looper.getMainLooper())
    private val inferenceInFlight = AtomicBoolean(false)

    private var running = false

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

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        statusText = findViewById(R.id.statusText)
        sampleText = findViewById(R.id.sampleText)
        outputText = findViewById(R.id.outputText)
        chart = findViewById(R.id.chart)

        setupChart()

        findViewById<Button>(R.id.startButton).setOnClickListener { startRealtimePipeline() }
        findViewById<Button>(R.id.stopButton).setOnClickListener { stopRealtimePipeline() }

        sensorManager = getSystemService(SENSOR_SERVICE) as SensorManager
        gyroSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        accSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        rotSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR)

        loadModel()
    }

    private fun setupChart() {
        trajectorySet = ScatterDataSet(mutableListOf(Entry(0f, 0f)), "Estimated Trajectory")
        trajectorySet.scatterShapeSize = 6f
        trajectorySet.color = 0xFFE53935.toInt()

        scatterData = ScatterData(trajectorySet)
        chart.data = scatterData
        chart.description.isEnabled = false
        chart.legend.isEnabled = true
        chart.setTouchEnabled(true)
        chart.setScaleEnabled(true)

        val xAxis = chart.xAxis
        xAxis.position = XAxis.XAxisPosition.BOTTOM
        chart.axisRight.isEnabled = false
    }

    private fun loadModel() {
        try {
            flexDelegate = FlexDelegate()
            val options = Interpreter.Options().addDelegate(flexDelegate)
            interpreter = Interpreter(loadModelFile(MODEL_FILE), options)
            statusText.text = "Modelo: $MODEL_FILE | Pronto"
            return
        } catch (t: Throwable) {
            // Alguns emuladores/dispositivos falham ao iniciar o delegate Flex nativo.
            flexDelegate?.close()
            flexDelegate = null
        }

        try {
            interpreter = Interpreter(loadModelFile(MODEL_FILE))
            statusText.text = "Modelo: $MODEL_FILE | Pronto (sem FlexDelegate)"
        } catch (t: Throwable) {
            statusText.text = "Falha ao carregar modelo: ${t.javaClass.simpleName}: ${t.message}"
        }
    }

    private fun loadModelFile(fileName: String): MappedByteBuffer {
        val fd: AssetFileDescriptor = assets.openFd(fileName)
        val input = FileInputStream(fd.fileDescriptor)
        val channel = input.channel
        return channel.map(FileChannel.MapMode.READ_ONLY, fd.startOffset, fd.declaredLength)
    }

    private fun startRealtimePipeline() {
        if (running) return
        if (interpreter == null) {
            statusText.text = "Modelo indisponivel"
            return
        }
        if (accSensor == null || gyroSensor == null || rotSensor == null) {
            statusText.text = "Sensores indisponiveis (acelerometro/giroscopio/rotacao)"
            return
        }
        resetPipelineState()
        running = true

        accSensor?.also { sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_FASTEST) }
        gyroSensor?.also { sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_FASTEST) }
        rotSensor?.also { sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_FASTEST) }

        statusText.text = "Modelo: $MODEL_FILE | Rodando"
    }

    private fun stopRealtimePipeline() {
        if (!running) return
        running = false
        sensorManager.unregisterListener(this)
        statusText.text = "Modelo: $MODEL_FILE | Parado"
    }

    private fun resetPipelineState() {
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
        inferenceInFlight.set(false)

        trajectorySet.clear()
        trajectorySet.addEntry(Entry(0f, 0f))
        scatterData.notifyDataChanged()
        chart.notifyDataSetChanged()
        chart.invalidate()

        sampleText.text = "Amostras: 0/200"
        outputText.text = "Saida: dx=0.0000 dy=0.0000"
    }

    override fun onSensorChanged(event: SensorEvent) {
        if (!running) return

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
                        // Fixa o frame inicial para que a orientacao inicial seja identidade.
                        initRotor = q.conjugate().normalized()
                    }
                }

                Sensor.TYPE_GYROSCOPE -> {
                    latestGyro = event.values.copyOfRange(0, 3)
                    hasGyro = true
                    processGyroTick(event.timestamp)
                }
            }
        } catch (t: Throwable) {
            stopRealtimePipeline()
            statusText.text = "Erro de sensor: ${t.javaClass.simpleName}: ${t.message}"
        }
    }

    private fun processGyroTick(timestampNs: Long) {
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

        appendWindow(orientedGyro, orientedAcc)
        sampleText.text = "Amostras: ${sampleCount.coerceAtMost(WINDOW_SIZE)}/200"

        if (sampleCount < WINDOW_SIZE) return

        inferenceTicker += 1
        if (inferenceTicker % INFER_EVERY_N_SAMPLES != 0) return
        if (!inferenceInFlight.compareAndSet(false, true)) return

        val input = snapshotInput()
        val dtForInference = dtMeanSec

        worker.execute {
            try {
                val output = Array(1) { FloatArray(2) }
                interpreter?.run(input, output)

                val dx = output[0][0] * dtForInference
                val dy = output[0][1] * dtForInference
                posX += dx
                posY += dy

                ui.post {
                    outputText.text = "Saida: dx=${"%.4f".format(dx)} dy=${"%.4f".format(dy)}"
                    trajectorySet.addEntry(Entry(posX, posY))
                    scatterData.notifyDataChanged()
                    chart.notifyDataSetChanged()
                    chart.invalidate()
                }
            } catch (t: Throwable) {
                ui.post {
                    stopRealtimePipeline()
                    statusText.text = "Falha na inferencia: ${t.javaClass.simpleName}: ${t.message}"
                }
            } finally {
                inferenceInFlight.set(false)
            }
        }
    }

    private fun appendWindow(gyro: FloatArray, acc: FloatArray) {
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

    private fun snapshotInput(): Array<Array<FloatArray>> {
        val input = Array(1) { Array(CHANNELS) { FloatArray(WINDOW_SIZE) } }
        for (c in 0 until CHANNELS) {
            System.arraycopy(window[c], 0, input[0][c], 0, WINDOW_SIZE)
        }
        return input
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    override fun onPause() {
        super.onPause()
        stopRealtimePipeline()
    }

    override fun onDestroy() {
        super.onDestroy()
        stopRealtimePipeline()
        interpreter?.close()
        flexDelegate?.close()
        worker.shutdownNow()
    }
}
