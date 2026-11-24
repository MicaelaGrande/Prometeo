# Los textos de la llave de modo (prometeo_navigation/modos.py), escritos UNA sola vez.
#
# Los topics "modo" y "modo_actual" siguen siendo std_msgs/String (asi el panel no
# depende de este paquete). Este mensaje solo sirve para tener los textos como
# constantes: en Python se usa Modo.PEDIR_AUTO, Modo.TERMINADO, etc., y si alguien
# escribe mal el nombre, Python da error al arrancar en vez de ignorarlo en silencio.
# Igual que Mover.Goal.IR_A_SALIDA o Lugar.IZQUIERDA.
#
# El panel (my_robot_bringup/web/panel.html) es JavaScript y no puede leer esto:
# ahi estan copiados una vez, arriba de todo. Si se cambia algo aca, cambiarlo alla.

# Estados de la llave (lo que avisa "modo_actual"). Una sola via:
#   MANUAL ──► AUTO ──► TERMINADO   (no se puede volver)
string MANUAL="manual"        # lo manejan a mano (panel o joystick)
string AUTO="auto"            # maneja la navegacion; el manejo a mano queda bloqueado
string TERMINADO="terminado"  # quieto para siempre: para volver a empezar, relanzar

# Pedidos (lo que se manda por "modo")
string PEDIR_AUTO="auto"      # pasar de manual a automatico (una sola vez)
string PEDIR_PARAR="parar"    # terminar la corrida (PARAR, o termino el algoritmo)

# No se usa por ahora (los topics son String). Esta porque un mensaje necesita un campo.
string valor
