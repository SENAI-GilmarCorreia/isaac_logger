import carb
import rclpy

from rclpy.node import Node
from std_srvs.srv import SetBool  # Exemplo de serviço, ajuste para o tipo que preferir
import omni

class SceneLoaderService(Node):
    def __init__(self):
        super().__init__('scene_loader_service')
    #     self.srv = self.create_service(SetBool, 'load_scene', self.load_scene_callback)
    
    # def load_scene_callback(self, request, response):
    #     scene_name = request.name  # Atributo 'name' enviado no request
    #     try:
    #         # Carrega a cena usando a API do Isaac Sim
    #         omni.usd.get_context().open_stage(scene_name)
    #         response.success = True
    #         response.message = f'Scene {scene_name} loaded successfully.'
    #     except Exception as e:
    #         response.success = False
    #         response.message = str(e)
    #     return response

class IsaacROS2Extension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[omni.isaac.ros2] Isaac ROS2 Extension Startup")

        # Iniciar rclpy e configurar o nó ROS
        rclpy.init()
        self.node = SceneLoaderService()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)

        # Rodar o executor do ROS em um thread separado
        import threading
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def on_shutdown(self):
        # Encerrar o nó ROS e o executor
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()

        print("[omni.isaac.ros2] Isaac ROS2 Extension Shutdown")
