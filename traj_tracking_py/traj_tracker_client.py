import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import csv
import sys


class TrajTrackerClient(Node):
    def __init__(self, csv_file, robot_namespace='/mobile_wx250s'):
        super().__init__('trajectory_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, robot_namespace + '/arm_controller/follow_joint_trajectory')
        self.csv_file = csv_file
        # Nomi dei giunti standard, usati sia per posizioni che per velocità
        self.joint_names = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']


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
        rclpy.shutdown()


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(f"Feedback: {feedback}") # mettendo .debug non mi compaiono troppi messaggi di feedback a terminale


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Uso: ros2 run traj_tracking_py traj_tracker_client.py path/to/trajectory.csv")
        return 
    csv_file = sys.argv[1]
    trajectory_client = TrajTrackerClient(csv_file)
    trajectory_client.send_goal()
    rclpy.spin(trajectory_client)
    trajectory_client.destroy_node()


if __name__ == '__main__':
    main()