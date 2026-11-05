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
    Gazebo (topics sin namespace):
        ros2 launch my_robot_bringup panel.launch.py ns:=''

    Abrir http://localhost:8082 (o http://<ip-de-esta-compu>:8082 desde otra compu).
    Puertos: 8082 pagina, 9092 rosbridge. No choca con test_motores (8081/9091) ni con
    el dashboard (8080/9090).

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
    ]

    topics = PythonExpression([
        "'[' + ','.join(['", prefijo, "' + t for t in ['/modo', '/modo_actual', '/cmd_vel_manual',"
        " '/orden_enviada', '/distancias', '/odom', '/navegacion/aviso', '/mapeo/libreta']]) + ']'",
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

    selector = Node(
        package='prometeo_navigation',
        executable='selector_modo',
        namespace=ns,
        output='screen',
        condition=IfCondition(LaunchConfiguration('con_selector')),
    )

    # Servidor de la pagina. La raiz redirige al panel con el namespace y el puerto.
    servidor = (
        'import sys, functools, http.server as h\n'
        'class H(h.SimpleHTTPRequestHandler):\n'
        '    def do_GET(self):\n'
        '        if self.path.split("?")[0] in ("", "/", "/index.html"):\n'
        '            self.send_response(302)\n'
        '            self.send_header("Location", "/panel.html?ws=" + sys.argv[3] + "&ns=" + sys.argv[4])\n'
        '            self.end_headers()\n'
        '        else:\n'
        '            super().do_GET()\n'
        'h.test(HandlerClass=functools.partial(H, directory=sys.argv[2]), port=int(sys.argv[1]))\n'
    )
    web = ExecuteProcess(
        cmd=['python3', '-c', servidor, LaunchConfiguration('web_port'), web_dir,
             LaunchConfiguration('ws_port'), ns],
        output='screen',
    )

    aviso = LogInfo(msg=['Panel: http://localhost:', LaunchConfiguration('web_port'),
                         '/  (namespace "', ns, '")'])

    return LaunchDescription(arguments + [rosbridge, selector, web, aviso])
