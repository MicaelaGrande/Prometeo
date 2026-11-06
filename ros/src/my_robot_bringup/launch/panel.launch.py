from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Panel de navegacion (T11) + selector de modo. Funciona SIN INTERNET.

    Robot real (firmware de navegacion_bajo, topics en /prometeo):
        ros2 launch my_robot_bringup panel.launch.py
    Gazebo (topics sin namespace), con gazebo.launch.py ... joystick:=false:
        ros2 launch my_robot_bringup panel.launch.py ns:=''

    Abrir http://localhost:8082 (o http://<ip-de-esta-compu>:8082 desde otra compu).
    Puertos: 8082 pagina, 9092 rosbridge, 8083 camara. No choca con test_motores
    (8081/9091) ni con el dashboard (8080/9090).

    Asistencia de la Cat. B (asistencia:=true, solo robot real): freno por obstaculo y
    centrado entre el selector y la ESP32, con config/asistencia_robot_real.yaml. En Gazebo
    no hace falta: gazebo.launch.py ya la corre (nodo provisorio).

    Arranca tambien el joystick Xbox (joystick:=false para no) y la camara (camara:=false
    para no). La camara se busca sola en la red (buscar_camara.py: prueba camara_ip si se
    pasa, y si no recorre la red buscando la que tiene la MAC camara_mac) y arranca
    camera_bridge. Botones del joystick en config/joystick_xbox.yaml.

    Este rosbridge no tiene permiso sobre cmd_vel: el panel maneja por cmd_vel_manual,
    asi todo pasa por el selector. Lo que sale a la ESP32 lo ve en orden_enviada.
    """
    web_dir = PathJoinSubstitution([FindPackageShare('my_robot_bringup'), 'web'])
    ns = LaunchConfiguration('ns')
    # "/prometeo" o "" segun el namespace, para armar los nombres de los topics
    prefijo = PythonExpression(["'/' + '", ns, "' if '", ns, "' else ''"])

    arguments = [
        DeclareLaunchArgument('ns', default_value='prometeo',
                              description="namespace del robot: 'prometeo' (real) o '' (Gazebo)"),
        DeclareLaunchArgument('web_port', default_value='8082', description='puerto de la pagina'),
        DeclareLaunchArgument('ws_port', default_value='9092', description='puerto de rosbridge'),
        DeclareLaunchArgument('con_selector', default_value='true',
                              description='arrancar tambien el selector de modo'),
        DeclareLaunchArgument('asistencia', default_value='false',
                              description='freno por obstaculo y centrado en la compu (robot real)'),
        DeclareLaunchArgument('joystick', default_value='true',
                              description='arrancar el joystick Xbox'),
        DeclareLaunchArgument('camara', default_value='true',
                              description='buscar la ESP32-CAM y mostrarla en el panel'),
        DeclareLaunchArgument('camara_ip', default_value='',
                              description='IP de la ESP32-CAM, si se sabe (si no, se busca)'),
        DeclareLaunchArgument('camara_mac', default_value='d4:e9:f4:8a:b6:68',
                              description="MAC de nuestra ESP32-CAM ('' = cualquier camara)"),
        DeclareLaunchArgument('vision', default_value='true',
                              description='arrancar el nodo de vision (dibuja lo que detecta)'),
        DeclareLaunchArgument(
            'calibracion',
            default_value=PathJoinSubstitution([FindPackageShare('my_robot_bringup'), 'config',
                                                'calibracion_camara_ov3660.npz']),
            description='calibracion de la ESP32-CAM (copia de firmware/esp32_cam/calibracion)'),
        DeclareLaunchArgument('cam_port', default_value='8083',
                              description='puerto del video para el panel'),
    ]

    topics = PythonExpression([
        "'[' + ','.join(['", prefijo, "' + t for t in ['/modo', '/modo_actual', '/cmd_vel_manual',"
        " '/orden_enviada', '/estado_selector', '/pedido_operador', '/seguridad_habilitada',"
        " '/distancias', '/odom', '/navegacion/aviso', '/mapeo/libreta',"
        " '/estado_motores', '/velocidad', '/velocidad_ruedas', '/freno_emergencia',"
        " '/servos/estado', '/brazo/cmd', '/joy', '/freno_asistencia', '/motores/liberar']]"
        " + ['/vision/detections']) + ']'",
    ])

    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_panel',
        parameters=[{
            'port': ParameterValue(LaunchConfiguration('ws_port'), value_type=int),
            'topics_glob': ParameterValue(topics, value_type=str),
            'services_glob': '[/__ninguno__]',
            'params_glob': '[/__ninguno__]',
        }],
        output='screen',
    )

    # Sin asistencia, el selector le manda directo a la ESP32 (cmd_vel). Con asistencia, la
    # manda a cmd_vel_seleccionado y la asistencia es la que publica cmd_vel.
    con_selector = LaunchConfiguration('con_selector')
    asistencia_si = LaunchConfiguration('asistencia')
    selector = [
        Node(package='prometeo_navigation', executable='selector_modo', namespace=ns,
             output='screen',
             condition=IfCondition(PythonExpression(
                 ["'", con_selector, "' == 'true' and '", asistencia_si, "' != 'true'"]))),
        Node(package='prometeo_navigation', executable='selector_modo', namespace=ns,
             output='screen', remappings=[('cmd_vel', 'cmd_vel_seleccionado')],
             condition=IfCondition(PythonExpression(
                 ["'", con_selector, "' == 'true' and '", asistencia_si, "' == 'true'"]))),
    ]

    # Asistencia (Cat. B): el mismo programa y la misma cuenta que en Gazebo, conectado al
    # robot real con remapeos (no se toca su codigo).
    con_asistencia = IfCondition(asistencia_si)
    asistencia = [
        # /prometeo/distancias -> /ultrasonido/izq, /der, /frente (Range, en metros)
        Node(package='prometeo_sensores', executable='distancias_a_range',
             remappings=[('/distancias', [prefijo, '/distancias'])],
             condition=con_asistencia),
        Node(package='provisorio_bajo_nivel_sim', executable='centrado_simulado',
             name='asistencia',
             parameters=[PathJoinSubstitution([FindPackageShare('my_robot_bringup'), 'config',
                                               'asistencia_robot_real.yaml'])],
             remappings=[
                 ('/cmd_vel', [prefijo, '/cmd_vel_seleccionado']),
                 ('/cmd_vel_corregido', [prefijo, '/cmd_vel']),
                 # su propio aviso, para no pisar el freno_emergencia de la ESP32
                 ('/freno_emergencia', [prefijo, '/freno_asistencia']),
                 # el mismo boton del panel prende y apaga los dos frenos
                 ('/seguridad_habilitada', [prefijo, '/seguridad_habilitada']),
             ],
             output='screen', condition=con_asistencia),
    ]

    # Joystick: game_controller_node lee el control (numeracion estandar), teleop_twist_joy
    # lo convierte en movimiento por cmd_vel_joy (pasa por el selector) y joystick_modo
    # mira los botones de modo y freno.
    config_joy = PathJoinSubstitution([FindPackageShare('my_robot_bringup'), 'config',
                                       'joystick_xbox.yaml'])
    con_joystick = IfCondition(LaunchConfiguration('joystick'))
    joystick = [
        Node(package='joy', executable='game_controller_node', namespace=ns,
             parameters=[config_joy], condition=con_joystick),
        Node(package='teleop_twist_joy', executable='teleop_node', namespace=ns,
             name='teleop_twist_joy_node', parameters=[config_joy],
             remappings=[('cmd_vel', 'cmd_vel_joy')], condition=con_joystick),
        Node(package='prometeo_navigation', executable='joystick_modo', namespace=ns,
             parameters=[config_joy], condition=con_joystick),
    ]

    con_camara = IfCondition(LaunchConfiguration('camara'))
    camara = [
        # Busca la ESP32-CAM en la red y arranca camera_bridge (-> /camera/image_raw)
        ExecuteProcess(
            cmd=['ros2', 'run', 'my_robot_bringup', 'buscar_camara.py',
                 ['camara_ip:=', LaunchConfiguration('camara_ip')],
                 ['camara_mac:=', LaunchConfiguration('camara_mac')],
                 ['calibracion:=', LaunchConfiguration('calibracion')]],
            output='screen', condition=con_camara,
        ),
        # Vision: detecta flechas, cubos y zonas y publica /vision/annotated con los recuadros
        # (igual que vision_real.launch.py)
        Node(
            package='huggin_vision', executable='object_recognition', name='vision_node',
            parameters=[PathJoinSubstitution([FindPackageShare('my_robot_bringup'), 'config',
                                              'vision.yaml'])],
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('camara'), "' == 'true' and '",
                LaunchConfiguration('vision'), "' == 'true'"])),
        ),
        # Le pasa la imagen al panel (prefiere la de vision, con los recuadros)
        Node(
            package='my_robot_bringup', executable='camara_panel.py', name='camara_panel',
            parameters=[{'puerto': ParameterValue(LaunchConfiguration('cam_port'), value_type=int)}],
            output='screen', condition=con_camara,
        ),
    ]

    # Servidor de la pagina. La raiz redirige al panel con el namespace y el puerto.
    servidor = (
        'import sys, functools, http.server as h\n'
        'class H(h.SimpleHTTPRequestHandler):\n'
        '    def do_GET(self):\n'
        '        if self.path.split("?")[0] in ("", "/", "/index.html"):\n'
        '            self.send_response(302)\n'
        '            self.send_header("Location", "/panel.html?ws=" + sys.argv[3] + "&ns=" + sys.argv[4] + "&cam=" + sys.argv[5])\n'
        '            self.end_headers()\n'
        '        else:\n'
        '            super().do_GET()\n'
        'h.test(HandlerClass=functools.partial(H, directory=sys.argv[2]), port=int(sys.argv[1]))\n'
    )
    web = ExecuteProcess(
        cmd=['python3', '-c', servidor, LaunchConfiguration('web_port'), web_dir,
             LaunchConfiguration('ws_port'), ns, LaunchConfiguration('cam_port')],
        output='screen',
    )

    aviso = LogInfo(msg=['Panel: http://localhost:', LaunchConfiguration('web_port'),
                         '/  (namespace "', ns, '")'])

    return LaunchDescription(arguments + [rosbridge, web, aviso] + selector + asistencia
                             + camara + joystick)
