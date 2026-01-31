"""
Ejemplo 5: Teleoperación por Teclado (Teleop)
---------------------------------------------
Permite controlar los 6 movimientos del robot SoftBot en tiempo real.
Usa lectura de teclado no bloqueante (estilo videojuego).q

Movimientos:
1. Inflar A      2. Inflar B      3. Inflar Ambas
4. Succionar A   5. Succionar B   6. Succionar Ambas

Autor: Diego Gerardo Sanchez Moreno
"""

import sys
import select
import termios
import tty
import os
import time

# Importar la interfaz del SoftBot
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import rclpy
from sdk.softbot_interface import SoftBot

# Configuración Inicial
TARGET_PRESSURE_POS = 15.0  # kPa inicial para inflado
TARGET_PRESSURE_NEG = -15.0 # kPa inicial para succión
STEP_SIZE = 1.0             # Incremento de presión por tecla
BOOST_PULSE_S = 0.15        # Duración del pulso boost

msg = """
🤖 CONTROL MANUAL SOFTBOT v1.0
---------------------------
Moverse:
   Q : Inflar A      W : Inflar B      E : Inflar AMBAS
   A : Succionar A   S : Succionar B   D : Succionar AMBAS

Funciones:
   ESPACIO : PARADA DE EMERGENCIA (STOP)
   + / -   : Aumentar/Disminuir Presión Objetivo
   B       : BOOST ON/OFF (válvula tanque)
   T       : Pulso BOOST corto

CTRL-C para salir
"""

def getKey():
    """Lee una sola tecla de la terminal sin presionar Enter"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_status(bot, action, chamber, target):
    """Muestra estado en una sola línea (limpio)"""
    state = bot.get_state()
    # Borrar línea actual y reescribir
    sys.stdout.write(f"\r\033[K🎮 Acción: {action} [{chamber}] | Objetivo: {target} kPa | Actual: {state['pressure']:.2f} kPa")
    sys.stdout.flush()

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    
    try:
        bot = SoftBot()
        
        # Reset inicial de seguridad
        bot.set_chamber(3) 
        bot.stop()
        
        print(msg)
        print(f"🎯 Presión Inflado Actual: {TARGET_PRESSURE_POS} kPa")
        print(f"🎯 Presión Succión Actual: {TARGET_PRESSURE_NEG} kPa\n")

        current_action = "STOP"
        current_chamber_name = "N/A"
        current_target = 0.0
        boost_on = False

        while True:
            key = getKey()
            
            # --- MAPA DE TECLAS ---
            
            # 1. MOVIMIENTOS INFLADO (Fila QWE)
            if key == 'q':
                bot.set_chamber(1)
                bot.inflate(TARGET_PRESSURE_POS)
                current_action = "INFLANDO"
                current_chamber_name = "A"
                current_target = TARGET_PRESSURE_POS

            elif key == 'w':
                bot.set_chamber(2)
                bot.inflate(TARGET_PRESSURE_POS)
                current_action = "INFLANDO"
                current_chamber_name = "B"
                current_target = TARGET_PRESSURE_POS

            elif key == 'e':
                bot.set_chamber(3)
                bot.inflate(TARGET_PRESSURE_POS)
                current_action = "INFLANDO"
                current_chamber_name = "A+B"
                current_target = TARGET_PRESSURE_POS

            # 2. MOVIMIENTOS SUCCIÓN (Fila ASD)
            elif key == 'a':
                bot.set_chamber(1)
                bot.suction(TARGET_PRESSURE_NEG)
                current_action = "SUCCIONANDO"
                current_chamber_name = "A"
                current_target = TARGET_PRESSURE_NEG

            elif key == 's':
                bot.set_chamber(2)
                bot.suction(TARGET_PRESSURE_NEG)
                current_action = "SUCCIONANDO"
                current_chamber_name = "B"
                current_target = TARGET_PRESSURE_NEG

            elif key == 'd':
                bot.set_chamber(3)
                bot.suction(TARGET_PRESSURE_NEG)
                current_action = "SUCCIONANDO"
                current_chamber_name = "A+B"
                current_target = TARGET_PRESSURE_NEG

            # 3. CONTROL Y AJUSTES
            elif key == ' ': # Espacio = STOP
                bot.stop()
                current_action = "STOP"
                current_chamber_name = "-"
                current_target = 0.0
                print(f"\n🛑 PARADA (Presión liberada)")

            elif key == '+':
                TARGET_PRESSURE_POS += STEP_SIZE
                TARGET_PRESSURE_NEG -= STEP_SIZE # Aumenta magnitud de vacío
                print(f"\n⬆️ Presión Ajustada: Inflar {TARGET_PRESSURE_POS} / Succión {TARGET_PRESSURE_NEG}")

            elif key == '-':
                TARGET_PRESSURE_POS -= STEP_SIZE
                TARGET_PRESSURE_NEG += STEP_SIZE 
                print(f"\n⬇️ Presión Ajustada: Inflar {TARGET_PRESSURE_POS} / Succión {TARGET_PRESSURE_NEG}")

            elif key == '\x03': # Ctrl+C
                break
            elif key in ('b', 'B'):
                boost_on = not boost_on
                bot.set_boost(boost_on)
                print(f\"\\n⚡ BOOST {'ON' if boost_on else 'OFF'}\")
            elif key in ('t', 'T'):
                bot.pulse_boost(BOOST_PULSE_S)
                print(f\"\\n⚡ PULSO BOOST {BOOST_PULSE_S:.2f}s\")

            # Actualizar feedback visual
            print_status(bot, current_action, current_chamber_name, current_target)
            time.sleep(0.05) # Pequeña pausa para no saturar CPU

    except Exception as e:
        print(e)

    finally:
        bot.stop()
        bot.close()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\n👋 Teleoperación finalizada.")
