"""
Ejemplo 3: Configuración de Seguridad en Tiempo Real
----------------------------------------------------
Herramienta CLI para modificar los límites de presión del firmware
sin necesidad de recompilar. Utiliza el tópico de Tuning Vectorial.
"""

import sys
import os
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import rclpy
from softbot_interface import SoftBot

def main():
    rclpy.init()
    bot = SoftBot()

    print("\n🛡️  GESTOR DE LÍMITES DE SEGURIDAD (SoftBot v5.0)")
    print("-------------------------------------------------")
    print("Nota: Estos cambios son volátiles (se pierden al reiniciar el ESP32)")
    
    try:
        while True:
            print(f"\nConfiguración Actual (Cache):")
            print(f"   Max: {bot.tuning_cache['max_safe']} kPa")
            print(f"   Min: {bot.tuning_cache['min_safe']} kPa")
            
            print("\nOpciones:")
            print("   1. Modificar Límite MÁXIMO (Sobrepresión)")
            print("   2. Modificar Límite MÍNIMO (Vacío)")
            print("   3. Salir")
            
            choice = input("Seleccione una opción: ")
            
            if choice == '1':
                val = float(input("   Nuevo valor MÁXIMO (kPa): "))
                bot.update_tuning(max_safe=val)
                time.sleep(0.5) # Esperar propagación
                print("   ✅ Comando enviado.")
                
            elif choice == '2':
                val = float(input("   Nuevo valor MÍNIMO (kPa): "))
                bot.update_tuning(min_safe=val)
                time.sleep(0.5)
                print("   ✅ Comando enviado.")
                
            elif choice == '3':
                break
            else:
                print("   ❌ Opción no válida.")

    except ValueError:
        print("\n❌ Error: Debe ingresar un número válido.")
    except KeyboardInterrupt:
        print("\nSalida forzada.")
    finally:
        bot.close()

if __name__ == '__main__':
    main()