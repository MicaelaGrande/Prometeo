import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, Float32MultiArray, String

from brazo_interfaces.srv import SolveIK
from brazo_kinematics.ik import ArmParams, solve


class IKNode(Node):
    """/brazo/target (PointStamped, m) -> IK -> /brazo/cmd [base, hombro, codo, garra] (grados)."""

    def __init__(self):
        super().__init__('brazo_ik')
        defaults = ArmParams()
        for name in ('L1', 'L2', 'h', 'd', 'e', 'c', 'gamma_min_deg', 'gamma_max_deg'):
            self.declare_parameter(name, getattr(defaults, name))
        for name in ('sign', 'offset_deg', 'servo_min_deg', 'servo_max_deg'):
            self.declare_parameter(name, list(getattr(defaults, name)))
        self.declare_parameter('frame_id', 'base_link')

        self.params = self._load_params()
        self.frame_id = self.get_parameter('frame_id').value
        self.garra = math.nan   # NaN = el firmware deja la garra como está

        self.pub_cmd = self.create_publisher(Float32MultiArray, 'brazo/cmd', 10)
        self.pub_status = self.create_publisher(String, 'brazo/ik_status', 10)
        self.create_subscription(PointStamped, 'brazo/target', self.on_target, 10)
        self.create_subscription(Float32, 'brazo/garra', self.on_garra, 10)
        self.create_service(SolveIK, 'brazo/solve_ik', self.on_solve)
        self.get_logger().info('IK listo: /brazo/target -> /brazo/cmd')

    def _load_params(self) -> ArmParams:
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        t = lambda n: tuple(float(v) for v in g(n))  # noqa: E731
        for n in ('sign', 'offset_deg', 'servo_min_deg', 'servo_max_deg'):
            if len(g(n)) != 3:
                raise ValueError(f'{n} necesita 3 valores')
        return ArmParams(
            L1=g('L1'), L2=g('L2'), h=g('h'), d=g('d'), e=g('e'), c=g('c'),
            gamma_min_deg=g('gamma_min_deg'), gamma_max_deg=g('gamma_max_deg'),
            sign=t('sign'), offset_deg=t('offset_deg'),
            servo_min_deg=t('servo_min_deg'), servo_max_deg=t('servo_max_deg'))

    def on_garra(self, msg: Float32):
        self.garra = float(msg.data)
        cmd = Float32MultiArray()
        cmd.data = [math.nan, math.nan, math.nan, self.garra]   # solo mueve la garra
        self.pub_cmd.publish(cmd)

    def on_target(self, msg: PointStamped):
        if msg.header.frame_id not in ('', self.frame_id):
            self.get_logger().warn(
                f"frame '{msg.header.frame_id}' != '{self.frame_id}' (no hay TF, se usa tal cual)",
                throttle_duration_sec=5.0)
        res = solve(self.params, msg.point.x, msg.point.y, msg.point.z)
        status = String()
        if res.ok:
            cmd = Float32MultiArray()
            cmd.data = [float(v) for v in res.servo_deg] + [self.garra]
            self.pub_cmd.publish(cmd)
            status.data = 'ok'
        else:
            status.data = f'rechazado: {res.message}'
            self.get_logger().warn(status.data)   # no se manda nada: el brazo no se mueve
        self.pub_status.publish(status)

    def on_solve(self, req, resp):
        res = solve(self.params, req.target.x, req.target.y, req.target.z)
        resp.ok = res.ok
        resp.message = res.message
        resp.servo_deg = [float(v) for v in res.servo_deg] if res.ok else []
        resp.elbow_interior_deg = float(math.degrees(res.gamma)) if res.ok else math.nan
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
