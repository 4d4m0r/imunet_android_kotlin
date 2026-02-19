# IMUNet Android Kotlin

Aplicativo Android em Kotlin para inferencia IMU em tempo real com IMUNet:

1. Le acelerometro, giroscopio e game rotation vector em `SENSOR_DELAY_FASTEST`.
2. Faz alinhamento de orientacao via quaternion.
3. Monta janela deslizante `input[1][6][200]`.
4. Executa `IMUNet.tflite` em tempo real.
5. Integra saida 2D e plota a trajetoria no grafico.

## Estrutura

- `app/src/main/kotlin/edu/usf/imunetkotlin/MainActivity.kt`: pipeline completo real-time.
- `app/src/main/kotlin/edu/usf/imunetkotlin/Quaternion.kt`: operacoes de quaternion.
- `app/src/main/assets/IMUNet.tflite`: modelo.

## Executar

No Android Studio, abra a pasta `imunet_android_kotlin` como projeto separado e rode `app`.

Se quiser rodar por terminal, configure antes:

- `ANDROID_HOME` ou `ANDROID_SDK_ROOT` apontando para seu SDK Android.

Depois:

```bash
./gradlew :app:assembleDebug
```
