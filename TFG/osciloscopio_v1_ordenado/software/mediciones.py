import numpy as np


def calcular_vmax(signal):
    """
    Calcula el valor máximo de la señal.
    """
    if len(signal) == 0:
        return 0.0

    return float(np.max(signal))


def calcular_vmin(signal):
    """
    Calcula el valor mínimo de la señal.
    """
    if len(signal) == 0:
        return 0.0

    return float(np.min(signal))


def calcular_vpp(signal):
    """
    Calcula el valor pico a pico de la señal.
    """
    if len(signal) == 0:
        return 0.0

    return calcular_vmax(signal) - calcular_vmin(signal)


def calcular_vmedio(signal):
    """
    Calcula el valor medio de la señal.
    """
    if len(signal) == 0:
        return 0.0

    return float(np.mean(signal))


def calcular_vrms(signal, quitar_dc=True):
    """
    Calcula el valor RMS de la señal.

    Si quitar_dc=True, primero elimina el valor medio.
    Esto sirve para calcular el RMS de la componente alterna.
    """
    if len(signal) == 0:
        return 0.0

    data = np.asarray(signal, dtype=np.float32)

    if quitar_dc:
        data = data - np.mean(data)

    return float(np.sqrt(np.mean(data ** 2)))


def calcular_frecuencia(signal, fs_hz):
    """
    Calcula la frecuencia de una señal usando cruces por cero ascendentes.

    Parámetros:
    - signal: muestras de tensión.
    - fs_hz: frecuencia de muestreo en Hz.

    Retorna:
    - Frecuencia estimada en Hz.
    """

    if len(signal) < 3:
        return 0.0

    data = np.asarray(signal, dtype=np.float32)

    # Quitamos offset DC para facilitar la detección de cruces por cero.
    data = data - np.mean(data)

    cruces = []

    for i in range(1, len(data)):
        if data[i - 1] < 0 and data[i] >= 0:
            # Interpolación lineal para estimar mejor el cruce.
            y1 = data[i - 1]
            y2 = data[i]

            if y2 != y1:
                frac = -y1 / (y2 - y1)
            else:
                frac = 0.0

            cruce = (i - 1) + frac
            cruces.append(cruce)

    if len(cruces) < 2:
        return 0.0

    periodos = np.diff(cruces)

    if len(periodos) == 0:
        return 0.0

    periodo_muestras = np.mean(periodos)

    if periodo_muestras <= 0:
        return 0.0

    frecuencia = fs_hz / periodo_muestras

    return float(frecuencia)


def calcular_mediciones(signal, fs_hz):
    """
    Calcula todas las mediciones principales de una señal.

    Retorna un diccionario con:
    - vmax
    - vmin
    - vpp
    - vmedio
    - vrms_ac
    - vrms_total
    - frecuencia
    """

    return {
        "vmax": calcular_vmax(signal),
        "vmin": calcular_vmin(signal),
        "vpp": calcular_vpp(signal),
        "vmedio": calcular_vmedio(signal),
        "vrms_ac": calcular_vrms(signal, quitar_dc=True),
        "vrms_total": calcular_vrms(signal, quitar_dc=False),
        "frecuencia": calcular_frecuencia(signal, fs_hz),
    }