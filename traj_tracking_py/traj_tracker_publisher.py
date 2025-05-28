import rclpy
from rclpy.node import Node
import csv
import sys
from interbotix_xs_msgs.msg import JointGroupCommand

class TrajTrackerPublisher(Node):
    def __init__(self, csv_file):
        super().__init__('traj_tracker_publisher')
        self.csv_file = csv_file
        # Leggi i punti dal CSV (logic simile fino a riga 34 del tuo client).
        self.points = self.read_csv_trajectory()

        # Crea un publisher sul topic /mobile_wx250s/commands/joint_group
        self.pub = self.create_publisher(JointGroupCommand, '/mobile_wx250s/commands/joint_group', 10)

        # Indice per scorrere i punti, saltando quelli intermedi
        self.current_index = 0
        # Avviso che il nodo è stato avviato
        self.get_logger().info("Inizio Traiettoria.")

        # Timer: pubblica ogni 0.1s (per esempio), puoi regolare il valore
        self.timer = self.create_timer(0.001, self.timer_cb)

    def read_csv_trajectory(self):
        points = []
        with open(self.csv_file, mode='r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                # Costruiamo il "punto" come un elenco di posizioni
                positions = []
                for joint in ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']:
                    positions.append(float(row[joint]))
                points.append(positions)
        return points

    def timer_cb(self):
        # Se abbiamo ancora punti da pubblicare
        if self.current_index < len(self.points):
            # Crea il messaggio
            msg = JointGroupCommand()
            msg.name = "arm"  # Nome del gruppo, adattalo se necessario
            msg.cmd = self.points[self.current_index]  # Posizioni di questo step
            # Pubblica il messaggio
            self.pub.publish(msg)
            # Avanza di 10
            self.current_index += 1
        else:
            # Non ci sono più punti, chiudi il nodo
            self.get_logger().info("Traiettoria completata.")
            self.destroy_timer(self.timer)  # Distruggi il timer
            rclpy.shutdown()

def main():
    rclpy.init()
    if len(sys.argv) < 2:
        print("Uso: ros2 run traj_tracking_py traj_tracker_publisher path/to/trajectory.csv")
        return

    csv_file = sys.argv[1]
    node = TrajTrackerPublisher(csv_file)
    rclpy.spin(node)
    node.destroy_node()
    #rclpy.shutdown()

if __name__ == '__main__':
    main()
