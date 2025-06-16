import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import csv
import sys
import time 
from interbotix_xs_msgs.srv import OperatingModes 


class TrajTrackerClient(Node):
    def __init__(self, csv_file, robot_namespace='/mobile_wx250s'):
        super().__init__('trajectory_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, robot_namespace + '/arm_controller/follow_joint_trajectory')
        self.csv_file = csv_file
        # Nomi dei giunti standard, usati sia per posizioni che per velocità
        self.joint_names = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
        # Client per il servizio di impostazione modalità operative
        self.op_mode_client = self.create_client(OperatingModes, robot_namespace + '/set_operating_modes')
        self.robot_namespace = robot_namespace

    def read_csv_trajectory(self):
        points = []
        with open(self.csv_file, mode='r') as file:
            reader = csv.DictReader(file)

            # Controlla se le colonne delle velocità sono presenti nell'header
            # Ci aspettiamo nomi di colonna tipo 'waist_d', 'shoulder_d', etc.
            has_velocities = True
            if reader.fieldnames:
                for joint_name in self.joint_names:
                    if f"{joint_name}_d" not in reader.fieldnames:
                        has_velocities = False
                        self.get_logger().info(f"Colonna velocità '{joint_name}_d' non trovata. Si assumerà che il CSV contenga solo posizioni.")
                        break
                if has_velocities:
                    self.get_logger().info("Rilevate colonne di velocità nel CSV.")
            else:
                # Nessun header o file vuoto
                has_velocities = False
                self.get_logger().warn("CSV senza header o vuoto. Impossibile determinare la presenza di velocità.")


            for row in reader:
                point = JointTrajectoryPoint()
                # Imposta time_from_start dal valore "time" del file
                time_sec = float(row['time']) 
                point.time_from_start.sec = int(time_sec)  # il campo sec dell'interfaccia deve essere un int! 
                # Dobbiamo gestire anche i nanosecondi
                point.time_from_start.nanosec = int((time_sec - int(time_sec)) * 1e9)

                # Estrae le posizioni per ciascun giunto
                positions = []
                for joint in self.joint_names:
                    positions.append(float(row[joint]))
                point.positions = positions

                if has_velocities:
                    velocities = []
                    for joint in self.joint_names:
                        velocities.append(float(row[f"{joint}_d"]))
                    point.velocities = velocities

                points.append(point)
        return points

    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()
        # Definisce l'elenco dei giunti, assicurarsi che corrispondano al CSV
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = self.read_csv_trajectory()

        self.get_logger().info("Invio del goal della traiettoria...")
        self._action_client.wait_for_server()
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Il goal è stato rifiutato")
            return

        self.get_logger().info("Il goal è stato accettato")

        # Ora attendi il risultato
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    # Callback per ricevere messaggio al completamento della traiettoria e per terminare il nodo
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Trajectory tracking terminato con successo")
        # Ripristina i profili motore
        self.get_logger().info("Ripristino profili motore (profile_type='time', vel=2000, accel=300)...")
        if self._set_robot_profiles(profile_type="time", profile_velocity=2000, profile_acceleration=300):
            self.get_logger().info("Profili motore ripristinati correttamente.")
        else:
            self.get_logger().warn("Fallimento nel ripristino dei profili motore.")
        
        rclpy.shutdown()



    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(f"Feedback: {feedback}") # mettendo .debug non mi compaiono troppi messaggi di feedback a terminale


    def go_to_home_position(self, home_positions=None, wait_for_result_timeout_sec=30.0):
        # Invia il robot a una posizione Home definita e attende il completamento
        if home_positions is None:
            home_positions = [0.0] * len(self.joint_names)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = home_positions
        # Tempo ragionevole per raggiungere la posizione Home
        point.time_from_start.sec = 3 
        point.time_from_start.nanosec = 0
        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f"Invio del goal per la posizione Home: {home_positions}...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"Action server '{self._action_client.action_name}' non disponibile per il goal Home.")
            return False

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        # Attendi che il goal sia inviato e processato (accettato/rifiutato)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)

        if not send_goal_future.done() or send_goal_future.result() is None:
            self.get_logger().error("Invio del goal 'Home' fallito o timeout.")
            return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal 'Home' rifiutato.")
            return False

        self.get_logger().info("Goal 'Home' accettato. In attesa del risultato...")
        result_future = goal_handle.get_result_async()
        
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=wait_for_result_timeout_sec)

        if not result_future.done() or result_future.result() is None:
            self.get_logger().error("Ottenimento del risultato del goal 'Home' fallito o timeout.")
            return False

        result = result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Posizione Home raggiunta con successo.")
            return True
        else:
            self.get_logger().error(f"Errore nel raggiungere la posizione Home: {result.error_string} (codice: {result.error_code})")
            return False


    def _set_robot_profiles(self, profile_type: str, profile_velocity: int, profile_acceleration: int, mode: str = "position", group_name: str = "arm") -> bool:
        """Chiama il servizio per impostare i profili operativi del robot."""
        self.get_logger().info(f"Tentativo di impostare profili: type='{profile_type}', prof_vel={profile_velocity}, prof_accel={profile_acceleration} per gruppo '{group_name}' in '{mode}' mode.")
        
        service_name = f"{self.robot_namespace}/set_operating_modes"
        if not self.op_mode_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"Servizio '{service_name}' non disponibile.")
            return False

        req = OperatingModes.Request()
        req.cmd_type = "group"
        req.name = group_name
        req.mode = mode
        req.profile_type = profile_type
        req.profile_velocity = profile_velocity
        req.profile_acceleration = profile_acceleration
        
        future = self.op_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.done():
            try:
                response = future.result()
                if response is not None: 
                    self.get_logger().info(f"Profili impostati correttamente tramite il servizio '{service_name}'.")
                    return True
                else:
                    self.get_logger().error(f"Errore nell'impostazione dei profili: il servizio '{service_name}' ha risposto None.")
                    return False
            except Exception as e:
                self.get_logger().error(f"Errore nell'impostazione dei profili: eccezione dal servizio '{service_name}': {e}")
                return False
        else: 
            self.get_logger().error(f"Timeout durante l'attesa della risposta dal servizio '{service_name}' per impostare i profili.")
            return False

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Uso: ros2 run traj_tracking_py traj_tracker_client.py path/to/trajectory.csv")
        return 
    csv_file = sys.argv[1]
    trajectory_client = TrajTrackerClient(csv_file)

    # 1. Andare alla posizione Home
    # Puoi definire una posizione Home specifica se necessario, es:
    # home_config = trajectory_client.points[1] Bisognerebbe far leggere il CSV prima per trovare la posizione Home     
    if trajectory_client.go_to_home_position(): # Usa default [0,...0]
        trajectory_client.get_logger().info("Robot in posizione Home.")
        time.sleep(2.0) # Attesa per stabilizzazione
    
    # 2. Impostare i profili di velocità e accelerazione a zero
    trajectory_client.get_logger().info("Impostazione profili motore (profile_type='time', vel=0, accel=0)...")
    if trajectory_client._set_robot_profiles(profile_type="time", profile_velocity=0, profile_acceleration=0):
        trajectory_client.get_logger().info("Profili motore impostati correttamente.")
        time.sleep(1) # Breve attesa per assicurare che i profili siano attivi

    # 3. Eseguire la traiettoria dal CSV
    trajectory_client.get_logger().info("Avvio esecuzione traiettoria da CSV...")
    trajectory_client.send_goal()
    rclpy.spin(trajectory_client)
    trajectory_client.destroy_node()


if __name__ == '__main__':
    main()