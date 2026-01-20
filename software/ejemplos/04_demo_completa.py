#!/usr/bin/env python3
"""
Ejemplo 4: Demo Completa para Video (Versión Extendida)
-------------------------------------------------------
Secuencia coreografiada para demostración de video.
Incluye reinicio de seguridad al inicio para evitar bloqueos
por pruebas anteriores.

Secuencia:
1. Inicialización y Reset (¡CRÍTICO!)
2. Hardware Check (Lazo Abierto + Turbo)
3. Control PID Preciso
4. Sintonización en Vivo (Tuning)
5. Prueba de Seguridad (E-STOP)
"""

import sys
import os
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import rclpy
from softbot_interface import SoftBot

def print_header(text):
    print("\n" + "="*60)
    print(f"🎬 ESCENA: {text}")
    print("="*60)

def esperar_usuario():
    input("\n👉 Presiona [ENTER] para acción... ")

def main():
    rclpy.init()
    bot = SoftBot()

    try:
        print_header("0. INICIALIZACIÓN Y RESET")
        print("⏳ Restaurando parámetros de fábrica para evitar bloqueos...")
        
        # --- PASO CRÍTICO: RESETEAR LÍMITES DE SEGURIDAD ---
        # Si la prueba anterior dejó el límite bajo (ej. 5.0), esto lo arregla.
        bot.update_tuning(
            max_safe=25.0, 
            min_safe=-25.0,
            kp_pos=12.0,
            ki_pos=300.0
        )
        bot.stop() # Asegurar modo 0
        time.sleep(1)
        print("✅ Sistema limpio y listo. Límites restaurados a 45/-60 kPa.")

        print("\n🤖 BIENVENIDO AL MODO DEMOSTRACIÓN")
        print("Prepara tu cámara.")
        esperar_usuario()

        # ---------------------------------------------------------
        # ESCENA 1: PRUEBA DE HARDWARE (LAZO ABIERTO)
        # ---------------------------------------------------------
        print_header("1. HARDWARE Y POTENCIA (OPEN LOOP)")
        
        print("1.1 [Cámara A] Inflado Manual al 50% (PWM 128)")
        print("    Duración: 5 segundos para apreciar el flujo.")
        esperar_usuario()
        bot.set_chamber(1)
        bot.set_pwm(2, 128)
        time.sleep(5) # Aumentado
        bot.stop()
        
        print("1.2 [Cámara B] Succión TURBO al 100% (PWM 255)")
        print("    (Deben encenderse Bomba Principal + Auxiliar)")
        esperar_usuario()
        bot.set_chamber(2)
        bot.set_pwm(-2, 255)
        time.sleep(6) # Aumentado
        bot.stop()

        # ---------------------------------------------------------
        # ESCENA 2: CONTROL AUTOMÁTICO (PID)
        # ---------------------------------------------------------
        print_header("2. CONTROL PID (LAZO CERRADO)")

        print("2.1 [Cámara A] Inflando a 15.0 kPa...")
        esperar_usuario()
        bot.set_chamber(1)
        bot.inflate(15.0)
        
        # Tiempo extendido para ver estabilización
        print("    Observando estabilización (8s)...")
        for i in range(8):
            state = bot.get_state()
            print(f"    T={i}s | Presión: {state['pressure']:.2f} kPa")
            time.sleep(1)
            
        print("\n2.2 [Cámara A] Succionando a -20.0 kPa...")
        esperar_usuario()
        bot.suction(-20.0)
        
        for i in range(8):
            state = bot.get_state()
            print(f"    T={i}s | Presión: {state['pressure']:.2f} kPa")
            time.sleep(1)
        
        bot.stop()

        # ---------------------------------------------------------
        # ESCENA 3: TUNING EN VIVO (ZERO-FLASH)
        # ---------------------------------------------------------
        print_header("3. SINTONIZACIÓN EN VIVO (ZERO-FLASH)")
        
        # A) PRUEBA SUAVE
        print("3.1 Inflado con KP Suave (12.0)")
        esperar_usuario()
        # Aseguramos valores base otra vez por si acaso
        bot.update_tuning(kp_pos=12.0) 
        bot.set_chamber(1)
        bot.inflate(15.0) 
        time.sleep(6)
        
        # B) RESET (DESINFLADO)
        print("\n    🔄 Desinflando para preparar contraste...")
        bot.suction(-5.0) 
        time.sleep(4)
        bot.stop()
        print("    Listo. Presión en 0.")
        
        # C) PRUEBA AGRESIVA
        print("\n3.2 ¡CAMBIANDO KP a 35.0! (Más agresivo)")
        print("    (El cambio es inmediato sin reiniciar el ESP32)")
        esperar_usuario()
        
        bot.update_tuning(kp_pos=35.0)
        print("    --> Tuning enviado. ¡Observa la reacción!")
        
        bot.inflate(15.0) 
        time.sleep(6)
        
        bot.stop()
        # Restaurar valor original
        bot.update_tuning(kp_pos=12.0)

        # ---------------------------------------------------------
        # ESCENA 4: SEGURIDAD (E-STOP)
        # ---------------------------------------------------------
        print_header("4. SISTEMA DE SEGURIDAD (E-STOP)")
        
        print("4.1 Generando presión estable (15 kPa)...")
        # Aseguramos límites seguros antes de empezar
        bot.update_tuning(max_safe=45.0) 
        bot.set_chamber(1)
        bot.inflate(15.0)
        time.sleep(5)
        
        print("\n4.2 SIMULANDO FALLA CRÍTICA")
        print("    Acción: Bajar límite de seguridad a 5 kPa")
        print("    Reacción esperada: VENTEO AUTOMÁTICO -> APAGADO TOTAL")
        esperar_usuario()
        
        bot.update_tuning(max_safe=5.0) # Forzar error
        
        print("    ⚠️ FALLA INYECTADA. Esperando reacción del firmware...")
        time.sleep(4) # Tiempo para ver el venteo
        
        state = bot.get_state()
        if state['mode'] == 0:
            print(f"\n✅ E-STOP CONFIRMADO. Presión final: {state['pressure']:.2f} kPa")
        else:
            print(f"\n⚠️ Estado actual: {state['mode']} (¿Se apagó?)")

        # Restaurar seguridad al final para dejar el robot listo
        print("\n4.3 Restaurando sistema para siguiente uso...")
        bot.update_tuning(max_safe=45.0)
        bot.stop()

        print("\n🎬 FIN DE LA DEMOSTRACIÓN")

    except KeyboardInterrupt:
        print("\nCancelado por usuario.")
        # Intento final de restaurar seguridad al salir forzadamente
        bot.update_tuning(max_safe=25.0)
        bot.stop()
    finally:
        bot.close()

if __name__ == '__main__':
    main()