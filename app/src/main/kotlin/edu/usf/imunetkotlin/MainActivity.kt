package edu.usf.imunetkotlin

import android.os.Bundle
import android.widget.Button
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import com.github.mikephil.charting.charts.ScatterChart
import com.github.mikephil.charting.components.XAxis
import com.github.mikephil.charting.data.Entry
import com.github.mikephil.charting.data.ScatterData
import com.github.mikephil.charting.data.ScatterDataSet
import edu.usf.imunetkotlin.pdr.PdrEngine
import edu.usf.imunetkotlin.pdr.PdrSnapshot

class MainActivity : AppCompatActivity() {

    private lateinit var statusText: TextView
    private lateinit var sampleText: TextView
    private lateinit var outputText: TextView
    private lateinit var chart: ScatterChart

    private lateinit var trajectorySet: ScatterDataSet
    private lateinit var scatterData: ScatterData
    private lateinit var pdrEngine: PdrEngine

    private var lastPlottedX = 0f
    private var lastPlottedY = 0f

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        statusText = findViewById(R.id.statusText)
        sampleText = findViewById(R.id.sampleText)
        outputText = findViewById(R.id.outputText)
        chart = findViewById(R.id.chart)

        setupChart()
        bindPdrEngine()
        bindActions()
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

    private fun bindPdrEngine() {
        pdrEngine = PdrEngine(applicationContext)
        pdrEngine.setListener { snapshot ->
            renderSnapshot(snapshot)
        }

        val ok = pdrEngine.initialize()
        pdrEngine.setEnable(ok)
    }

    private fun bindActions() {
        findViewById<Button>(R.id.startButton).setOnClickListener {
            pdrEngine.setEnable(true)
            pdrEngine.setStart(true)
        }
        findViewById<Button>(R.id.stopButton).setOnClickListener {
            pdrEngine.setStart(false)
        }
    }

    private fun renderSnapshot(snapshot: PdrSnapshot) {
        statusText.text = "${snapshot.status} | t=${snapshot.durationMillis}ms"
        sampleText.text = "Amostras: ${snapshot.sampleCount}/200"
        outputText.text =
            "Saida: dx=${"%.4f".format(snapshot.lastDx)} dy=${"%.4f".format(snapshot.lastDy)} | " +
                "dist=${"%.3f".format(snapshot.distanceMeters)}m | steps=${snapshot.trajectory.steps_count}"

        val x = snapshot.trajectory.rel_x
        val y = snapshot.trajectory.rel_y
        if (x != lastPlottedX || y != lastPlottedY) {
            trajectorySet.addEntry(Entry(x, y))
            scatterData.notifyDataChanged()
            chart.notifyDataSetChanged()
            chart.invalidate()
            lastPlottedX = x
            lastPlottedY = y
        }
    }

    override fun onPause() {
        super.onPause()
        pdrEngine.setStart(false)
    }

    override fun onDestroy() {
        super.onDestroy()
        pdrEngine.release()
    }
}
