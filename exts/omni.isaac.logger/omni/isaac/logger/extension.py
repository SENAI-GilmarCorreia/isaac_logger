import os
from datetime import datetime
import carb
import time
import csv
import omni.ext
import omni.timeline
import omni.physx
import omni.ui as ui
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core import SimulationContext
from omni.isaac.core import World

from pxr import Usd, UsdGeom, Sdf

# Isaac Logger Extension

class IsaacLogger():
    def __init__(self, timeline):

        if 'NAAD_WS_DIR' not in os.environ:
            raise KeyError("Environment variable 'NAAD_WS_DIR' is not set.")
        
        # Configurar o caminho do arquivo de log
        if 'NAAD_CONFIG_LOGS' in os.environ:
            naad_path = os.path.join(os.environ['NAAD_WS_DIR'], "logs")
            root_folder = os.path.join(naad_path, self.get_latest_folder(naad_path))
            file_name = 'isaacLogger.csv'
        else:
            root_folder = os.path.join(os.environ['NAAD_WS_DIR'], "logs")
            file_name = f'isaacLogger_{self.get_current_datetime()}.csv'

        self.sep = ';'
        logs_dir = os.path.join(root_folder, "isaac")
        os.makedirs(logs_dir, exist_ok=True)
        self.log_file = os.path.join(logs_dir, file_name)

        # Inicializar contexto da simulação
        self.timeline = timeline
        if not self.timeline:
            carb.log_error("Timeline interface not found")
            return
        
        self.simulation_context = SimulationContext.instance()
        self.stage = omni.usd.get_context().get_stage()

        # Variáveis de tempo e controle
        self.frame_count = 0
        self.total_frames = 0
        self.last_time = time.time()
        self.initial_time = time.time()
        self.fps = 0.0

        # Abre o arquivo CSV e mantém aberto até o fim da simulação
        self.csv_file = open(self.log_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file, delimiter=self.sep)

        # Escrever o cabeçalho
        self.csv_writer.writerow([
            "Timestamp (%Y-%m-%d_%H-%M-%S)",
            "Frame",
            "IsaacSim - Step Size (ms)",
            "IsaacSim - Simulation Time (ms)",
            "IsaacSim - Real Time (ms)",
            "OS - System Time (ms)",
            "IsaacSim - RTF",
            "OS - RTF",
            "IsaacSim - Render FPS (Hz)",
            "OS - Plugin FPS (Hz)",
            "Active Objects"
        ])

    ########### MÉTODOS ###########

    def get_latest_folder(self, path):
        # Obtém a lista de diretórios dentro da pasta especificada
        folders = [f for f in os.listdir(path) if os.path.isdir(os.path.join(path, f))]
        # Ordena os diretórios pela data de criação (ctime)
        latest_folder = max(folders, key=lambda f: os.path.getctime(os.path.join(path, f)))
        return latest_folder

    def get_current_datetime(self):
        return datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    def collect_metrics(self, event):
        current_time = time.time()
        elapsed_time = current_time - self.last_time

        if elapsed_time >= 1.0:
            self.fps = self.frame_count / elapsed_time
            self.frame_count = 0
            self.last_time = current_time

        self.frame_count += 1
        self.total_frames += 1

        #Get simulation step size (in ms)
        # if self.simulation_context.is_simulating():
        #     step_size = self.simulation_context.get_physics_dt() * 1000  # Convert to milliseconds
        # else:
        step_size = 0

        # Get simulation time
        sim_time = self.simulation_context.current_time
        
        # Get active objects in the scene
        objects_data = []

        prim = self.stage.GetPrimAtPath("/World")
        # Check if the prim exists
        if prim.IsValid():
            for child in prim.GetChildren():
                child_name = child.GetName()

                # Get the child prim
                child_prim = self.stage.GetPrimAtPath(child.GetPath())

                # Get the transform matrix
                xformable = UsdGeom.Xformable(child_prim)
                transform_matrix = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())

                # Convert transform matrix to rotation quaternion
                rotation_quat = transform_matrix.ExtractRotationQuat()
                position = transform_matrix.ExtractTranslation()

                # Extract the quaternion components
                quat_x, quat_y, quat_z, quat_w = rotation_quat.GetReal(), rotation_quat.GetImaginary()[0], rotation_quat.GetImaginary()[1], rotation_quat.GetImaginary()[2]
                
                pose = [position[0], position[1], position[2], quat_x, quat_y, quat_z, quat_w]

                objects_data.append({"alias": child_name, "pose": pose})
        else:
            print("Prim does not exist at the specified path.")

        print([
                self.get_current_datetime(),
                self.total_frames, 
                step_size, 
                sim_time * 1000,
                self.timeline.get_current_time() * 1000, 
                (current_time - self.initial_time) * 1000,
                None,
                None,
                1/self.simulation_context.get_rendering_dt(),
                self.fps, 
                objects_data
            ])
        # Log to CSV
        # if self.csv_file:
        #     writer = csv.writer(self.csv_file, delimiter=self.sep)
        #     writer.writerow([
        #         self.get_current_datetime(), 
        #         self.total_frames, 
        #         step_size, 
        #         sim_time * 1000,
        #         current_time * 1000, 
        #         (current_time - self.initial_time) * 1000,
        #         None,
        #         None,
        #         1/self.simulation_context.get_rendering_dt(),
        #         self.fps, 
        #         objects_data
        #     ])
            
    def stop_logging(self):
        # Fechar o arquivo CSV ao terminar
        self.csv_file.close()

# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class IsaacLoggerExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[isaac.logger] Isaac Logger startup")

        # Get the timeline interface
        self.timeline = omni.timeline.get_timeline_interface()
        if not self.timeline:
            carb.log_error("Timeline interface not found")
            return

        # Initialize the logger
        self.logger = None

        # Register the update callback to check the simulation time continuously
        self._update_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_update)

    def _on_update(self, event):
        # Check if the simulation is playing
        if self.timeline.is_playing() and self.logger == None:
            self.logger = IsaacLogger(self.timeline)

        elif self.timeline.is_playing() and self.logger != None:
            self.logger.collect_metrics(event)

        elif not self.timeline.is_playing() and self.logger != None:
            self.logger.stop_logging()
            self.logger = None

    def on_shutdown(self):
        if self.logger != None:
            self.logger.stop_logging()
            self.logger = None

        print("[isaac.logger] isaac logger shutdown")
        
