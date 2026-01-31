"""
Ejemplo 4: Demo Completa + Live View para Screen Recording
----------------------------------------------------------
Secuencia coreografiada para demostración.
Características v8.1:
- Usa cámara USB externa por by-id (Icatchtek).
- Abre una ventana flotante con el video en vivo y subtítulos.
- Ideal para grabar pantalla (Split Screen: Terminal + Video).
- Reset automático a 0 entre escenas.
- Telemetría en terminal.
"""

import sys
import os
import time
import threading
from datetime import datetime

# Importar OpenCV
try:
    import cv2
except ImportError:
    print("❌ Error: Necesitas instalar opencv. Ejecuta: pip install opencv-python")
    sys.exit(1)

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import rclpy
from sdk.softbot_interface import SoftBot

# --- CLASE DE GRABACIÓN Y VISUALIZACIÓN ---
class WebcamRecorder:
    def __init__(
        self,
        filename='demo_video.mp4',
        camera_device="/dev/v4l/by-id/usb-Icatchtek_Co_Ltd_General_Image_Device-video-index0"
    ):
        # Abrir cámara USB externa explícita usando V4L2
        self.cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)

        # Fallback a cámara 0 si falla
        if not self.cap.isOpened():
            print(f"⚠️ No se pudo abrir {camera_device}. Intentando con cámara 0...")
            self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            print("❌ ERROR CRÍTICO: No se detectó ninguna cámara.")
            self.valid = False
            return

        self.valid = True
        self.recording = False
        self.current_label = "Esperando inicio..."
        self.filename = filename

        # Configuración de Video
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps = 20.0

        # Codec mp4v
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.writer = cv2.VideoWriter(
            filename, fourcc, self.fps, (self.width, self.height)
        )

        # Hilo de grabación
        self.thread = threading.Thread(target=self._record_loop, daemon=True)

    def start(self):
        if not self.valid:
            return
        self.recording = True
        self.thread.start()
        print(f"🎥 Ventana de video abierta. Grabando respaldo en: {self.filename}")

    def stop(self):
        if not self.valid:
            return
        self.recording = False
        if self.thread.is_alive():
            self.thread.join()
        self.cap.release()
        self.writer.release()
        cv2.destroyAllWindows()
        print("🎥 Video cerrado.")

    def set_label(self, text):
        self.current_label = text

    def _record_loop(self):
        while self.recording:
            ret, frame = self.cap.read()
            if ret:
                # Barra negra superior
                cv2.rectangle(frame, (0, 0), (self.width, 80), (0, 0, 0), -1)

                color = (0, 255, 0)
                if "RESET" in self.current_label or "E-STOP" in self.current_label:
                    color = (0, 255, 255)
                elif "FALLA" in self.current_label:
                    color = (0, 0, 255)

                cv2.putText(
                    frame,
                    self.current_label,
                    (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    color,
                    2,
                )

                ts = datetime.now().strftime("%H:%M:%S")
                cv2.putText(
                    frame,
                    ts,
                    (self.width - 150, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (150, 150, 150),
                    1,
                )

                cv2.imshow(
                    "SoftBot Live Monitor (Graba tu pantalla ahora)", frame
                )
                cv2.waitKey(1)

                self.writer.write(frame)
            else:
                time.sleep(0.1)

# --- VARIABLES GLOBALES ---
recorder = None

def print_header(text):
    print("\n" + "=" * 60)
    print(f"🎬 ESCENA: {text}")
    print("=" * 60)
    if recorder:
        recorder.set_label(text)

def esperar_usuario(msg="\n👉 Presiona [ENTER] para continuar... "):
    if recorder:
        recorder.set_label("PAUSA: Esperando Usuario...")
    input(msg)

def reset_robot_state(bot):
    if recorder:
        recorder.set_label("RESET: Reset del Robot...")
    print("\n    🔄 [RESET] Limpiando aire residual (3s)...")
    bot.set_chamber(3)
    bot.suction(-5.0)

    for _ in range(3):
        time.sleep(1)

    bot.stop()
    print("    ✅ Robot listo.")

def monitor_loop(bot, duration_sec, scene_name=""):
    if recorder:
        recorder.set_label(scene_name)
    for i in range(duration_sec):
        state = bot.get_state()
        print(
            f"    T={i+1}s | P: {state['pressure']:.2f} kPa | PWM: {state['pwm_main']}"
        )
        time.sleep(1)

def main():
    global recorder
    rclpy.init()
    bot = SoftBot()

    video_file = f"backup_{datetime.now().strftime('%H%M%S')}.mp4"

    print("🎥 Inicializando cámara USB externa (Icatchtek by-id)...")
    recorder = WebcamRecorder(
        filename=video_file,
        camera_device="/dev/v4l/by-id/usb-Icatchtek_Co_Ltd_General_Image_Device-video-index0"
    )
    recorder.start()

    try:
        recorder.set_label("CONFIGURANDO SETUP...")
        print("\n🎥 VENTANA DE VIDEO ABIERTA")
        print("💡 TIP: Acomoda esta terminal a la izquierda y el video a la derecha.")
        input("👉 Presiona ENTER para iniciar la Demo...")

        print_header("0. INICIALIZACIÓN")
        bot.update_tuning(
            max_safe=45.0,
            min_safe=-60.0,
            kp_pos=12.0,
            ki_pos=300.0,
            kp_neg=-75.0,
            ki_neg=-750.0,
        )
        reset_robot_state(bot)

        print_header("1. HARDWARE (OPEN LOOP)")

        print("1.1 Inflado Manual 50%")
        esperar_usuario()
        bot.set_chamber(1)
        bot.set_pwm(2, 128)
        monitor_loop(bot, 5, "1.1 HARDWARE: Inflado 50%")
        bot.stop()

        print("\n1.2 Succión TURBO 100%")
        esperar_usuario()
        bot.set_chamber(1)
        bot.set_pwm(-2, 255)
        monitor_loop(bot, 5, "1.2 HARDWARE: Turbo Succion 100%")
        bot.stop()

        reset_robot_state(bot)

        print_header("2. CONTROL PID")

        print("2.1 Inflar a 15.0 kPa")
        esperar_usuario()
        bot.set_chamber(2)
        bot.inflate(15.0)
        monitor_loop(bot, 8, "2.1 PI: Objetivo 15 kPa")

        print("\n2.2 Succionar a -15.0 kPa")
        esperar_usuario()
        bot.suction(-15)
        monitor_loop(bot, 6, "2.2 PI: Objetivo -15Pa")
        bot.stop()

        reset_robot_state(bot)

        print_header("3. TUNING EN VIVO")

        print("3.1 PI Suave (Kp=12)")
        esperar_usuario()
        bot.update_tuning(kp_pos=12.0)
        bot.set_chamber(1)
        bot.inflate(15.0)
        monitor_loop(bot, 6, "3.1 TUNING: Suave (Kp=12)")
        bot.stop()

        reset_robot_state(bot)

        print("\n3.2 PI Agresivo (Kp=25)")
        esperar_usuario()
        bot.set_chamber(1)
        bot.update_tuning(kp_pos=25.0)
        bot.inflate(15.0)
        monitor_loop(bot, 6, "3.2 TUNING: Agresivo (Kp=25)")
        bot.stop()

        bot.update_tuning(kp_pos=12.0)
        reset_robot_state(bot)

        print_header("4. E-STOP AUTOMATICO")

        bot.update_tuning(max_safe=45.0)
        bot.set_chamber(1)
        bot.inflate(15.0)
        monitor_loop(bot, 5, "4.1 Preparando E-STOP")

        esperar_usuario("\n👉 Presiona ENTER para romper el límite...")
        bot.update_tuning(max_safe=5.0)
        monitor_loop(bot, 4, "4.2 E-STOP ACTIVADO")

        bot.update_tuning(max_safe=45.0)
        reset_robot_state(bot)

        recorder.set_label("FIN DE LA DEMO")
        time.sleep(3)

    except KeyboardInterrupt:
        bot.update_tuning(max_safe=45.0, kp_pos=12.0)
        bot.stop()
    finally:
        bot.close()
        recorder.stop()

if __name__ == "__main__":
    main()
