from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Panel web del firmware de TEST DE MOTORES (ver micro_ros/main/main_test_motores.c).

    Abrir http://<ip-de-esta-maquina>:8081/test_motores.html
    Puertos: 8081 página, 9091 rosbridge. No choca con dashboard.launch.py (8080/9090).

    Este rosbridge SÍ deja publicar, pero solo en motores_test/cmd; /cmd_vel y el resto
    de topics, servicios y parámetros quedan rechazados.
    """
    web_dir = PathJoinSubstitution([FindPackageShare('my_robot_bringup'), 'web'])

    arguments = [
        DeclareLaunchArgument('web_port', default_value='8081', description='puerto de la página'),
        DeclareLaunchArgument('ws_port', default_value='9091', description='puerto de rosbridge'),
    ]

    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_test_motores',
        parameters=[{
            'port': ParameterValue(LaunchConfiguration('ws_port'), value_type=int),
            'topics_glob': '[/prometeo/motores_test/estado,/prometeo/motores_test/cmd]',
            'services_glob': '[/prometeo/__ninguno__]',
            'params_glob': '[/prometeo/__ninguno__]',
        }],
        output='screen',
    )

    # La carpeta web también tiene el dashboard (index.html, puerto 9090): la raíz
    # redirige al panel de test para no abrir la página equivocada.
    servidor = (
        'import sys, functools, http.server as h\n'
        'class H(h.SimpleHTTPRequestHandler):\n'
        '    def do_GET(self):\n'
        '        if self.path.split("?")[0] in ("", "/", "/index.html"):\n'
        '            self.send_response(302)\n'
        '            self.send_header("Location", "/test_motores.html?ws=" + sys.argv[3])\n'
        '            self.end_headers()\n'
        '        else:\n'
        '            super().do_GET()\n'
        'h.test(HandlerClass=functools.partial(H, directory=sys.argv[2]), port=int(sys.argv[1]))\n'
    )
    web = ExecuteProcess(
        cmd=['python3', '-c', servidor, LaunchConfiguration('web_port'), web_dir,
             LaunchConfiguration('ws_port')],
        output='screen',
    )

    aviso = LogInfo(msg=['Panel: http://localhost:', LaunchConfiguration('web_port'),
                         '/  (rosbridge en el puerto ', LaunchConfiguration('ws_port'), ')'])

    return LaunchDescription(arguments + [rosbridge, web, aviso])
