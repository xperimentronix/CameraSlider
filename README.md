# 🎥 CameraSlider

**Slider DIY motorizado para cámara con Arduino Nano y pantalla táctil Nextion**

Sketch de Arduino y programación de la pantalla Nextion para el slider de cámara de vídeo.

![Arduino](https://img.shields.io/badge/Arduino-Nano-00979D?logo=arduino&logoColor=white)
![Nextion](https://img.shields.io/badge/Pantalla-Nextion-2C3E50)
![Impresión 3D](https://img.shields.io/badge/Piezas-Impresión%203D-orange)
![DIY](https://img.shields.io/badge/Proyecto-DIY-brightgreen)

## 📺 Vídeo del proyecto

[![Mira el vídeo en YouTube](https://img.youtube.com/vi/w3cFDBiQwkQ/maxresdefault.jpg)](https://youtu.be/w3cFDBiQwkQ)

👆 *Haz clic en la imagen para ver el vídeo en YouTube*

## ✨ Qué hace

El slider mueve la cámara o el móvil en **dos ejes: desplazamiento y giro**. Combinando los dos se pueden hacer travellings y movimientos orbitales alrededor de un objeto.

- 🎛️ Velocidad y aceleración ajustables para conseguir movimientos suaves
- 🔇 Drivers TMC2208 silenciosos, para que el movimiento no tenga vibraciones y haga poco ruido
- 📍 Calibración de la posición inicial de los ejes
- 💾 Hasta **4 movimientos programables** (posición inicial, velocidad, aceleración y posición final)
- ▶️ Cada movimiento se puede probar, guardar y reproducir

## 🛠️ Cómo está hecho

| Parte | Componentes |
|---|---|
| **Rail** | Perfil de aluminio 20x40 V-Slot con patín de ruedas V-Slot |
| **Desplazamiento** | Motor paso a paso NEMA 17 + tornillo T8 (lead screw) de 8 mm |
| **Giro** | Motor paso a paso + correa y poleas GT2 |
| **Control** | Arduino Nano + CNC Shield V4 (modificada) + 2x drivers TMC2208 |
| **Interfaz** | Pantalla táctil Nextion |
| **Piezas** | Soporte giratorio y piezas impresas en 3D |

La lista de material completa, con enlaces de compra, está en la descripción del vídeo.

## 📂 Contenido del repositorio

- **Sketch de Arduino**: firmware del Arduino Nano que controla los motores
- **Proyecto Nextion**: interfaz de la pantalla táctil

## 🚀 Cómo cargar el programa

1. **Arduino**: abre el sketch con el IDE de Arduino y cárgalo en el Arduino Nano.
2. **Pantalla Nextion**: copia el fichero `.tft` a una tarjeta microSD, insértala en la pantalla y aliméntala. El programa se carga solo. Cuando termine, retira la tarjeta.

> [!IMPORTANT]
> Los drivers deben ser **TMC2208** para evitar ruido y vibraciones en la cámara. Además, para que funcionen hay que hacer una **modificación en la CNC Shield V4**, que explico en este vídeo:
> 👉 [Control de Arduino con pantalla Nextion](https://youtu.be/Op4_x1A6TdE)
>
> Para cargar el programa no hace falta entender todo ese vídeo: basta con ver el procedimiento para cargar el programa en la Nextion con una tarjeta SD.

## 🖨️ Piezas 3D

Ficheros STL: **¡Pronto en Thingiverse!**
