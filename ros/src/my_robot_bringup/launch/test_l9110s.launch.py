from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Panel web del firmware de TEST DEL DRIVER L9110S (ver micro_ros/main/main_test_l9110s.c).

    Abrir http://<ip-de-esta-maquina>:8082/test_l9110s.html
    Puertos: 8082 página, 9092 rosbridge. No choca con test_motores.launch.py (8081/9091)
    ni con dashboard.launch.py (8080/9090).

    Este rosbridge SÍ deja publicar, pero solo en l9110s_test/cmd; /cmd_vel y el resto
    de topics, servicios y parámetros quedan rechazados.
    """
    web_dir = PathJoinSubstitution([FindPackageShare('my_robot_bringup'), 'web'])

    arguments = [
        DeclareLaunchArgument('web_port', default_value='8082', description='puerto de la página'),
        DeclareLaunchArgument('ws_port', default_value='9092', description='puerto de rosbridge'),
    ]

    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_test_l9110s',
        parameters=[{
            'port': ParameterValue(LaunchConfiguration('ws_port'), value_type=int),
            'topics_glob': ('[/prometeo/l9110s_test/estado,/prometeo/l9110s_test/log,'
                            '/prometeo/l9110s_test/cmd]'),
            'services_glob': '[/prometeo/__ninguno__]',
            'params_glob': '[/prometeo/__ninguno__]',
        }],
        output='screen',
    )

    # La carpeta web tiene más de una página: la raíz redirige al panel de test
    servidor = (
        'import sys, functools, http.server as h\n'
        'class H(h.SimpleHTTPRequestHandler):\n'
        '    def do_GET(self):\n'
        '        if self.path.split("?")[0] in ("", "/", "/index.html"):\n'
        '            self.send_response(302)\n'
        '            self.send_header("Location", "/test_l9110s.html?ws=" + sys.argv[3])\n'
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
