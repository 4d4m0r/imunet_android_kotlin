package edu.usf.imunetkotlin

import android.hardware.SensorManager
import kotlin.math.sqrt

data class Quaternion(val x: Float, val y: Float, val z: Float, val w: Float) {

    fun normalized(): Quaternion {
        val n = sqrt((x * x + y * y + z * z + w * w).toDouble()).toFloat()
        if (n == 0f) return Quaternion(0f, 0f, 0f, 1f)
        return Quaternion(x / n, y / n, z / n, w / n)
    }

    fun conjugate(): Quaternion = Quaternion(-x, -y, -z, w)

    operator fun times(other: Quaternion): Quaternion {
        val nx = w * other.x + x * other.w + y * other.z - z * other.y
        val ny = w * other.y - x * other.z + y * other.w + z * other.x
        val nz = w * other.z + x * other.y - y * other.x + z * other.w
        val nw = w * other.w - x * other.x - y * other.y - z * other.z
        return Quaternion(nx, ny, nz, nw)
    }

    fun rotateVector(v: FloatArray): FloatArray {
        val p = Quaternion(v[0], v[1], v[2], 0f)
        val q = normalized()
        val r = q * p * q.conjugate()
        return floatArrayOf(r.x, r.y, r.z)
    }

    companion object {
        fun fromRotationVector(values: FloatArray): Quaternion {
            val q = FloatArray(4)
            SensorManager.getQuaternionFromVector(q, values)
            // Android retorna [w, x, y, z].
            return Quaternion(q[1], q[2], q[3], q[0]).normalized()
        }
    }
}
