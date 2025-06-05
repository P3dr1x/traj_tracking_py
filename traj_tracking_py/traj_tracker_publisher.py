import rclpy
from rclpy.node import Node
import csv
import sys
import time
from interbotix_xs_msgs.msg import JointGroupCommand
from interbotix_xs_msgs.srv import OperatingModes

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
            #self.destroy_timer(self.timer)  # Distruggi il timer
            self.destroy_node()
            rclpy.shutdown()

def main():
    rclpy.init()
    if len(sys.argv) < 2:
        print("Uso: ros2 run traj_tracking_py traj_tracker_publisher path/to/trajectory.csv")
        return

    csv_file = sys.argv[1]
    node = TrajTrackerPublisher(csv_file)

    # 1. Comando "Home": invio un messaggio (congiunto a zero) per mandare il robot nella configurazione Home.
    home_msg = JointGroupCommand()
    home_msg.name = "arm"  # assicurati che corrisponda al gruppo controllato
    #home_msg.cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # configurazione Home: adatta se necessario
    home_msg.cmd = node.read_csv_trajectory()[1]  # configurazione Home: adatta se necessario
    node.get_logger().info("Vado alla configurazione di partenza della traiettoria")
    node.pub.publish(home_msg)
    time.sleep(2.0)  # attesa per garantire il raggiungimento della posizione Home   

    # 2. Impostazione dei profili di velocità e accelerazione a zero
    # tramite una chiamata al service /mobile_wx250s/set_operating_modes.
    node.get_logger().info("Impostazione profili (profile_type='time', velocità e accelerazione = 0)...")
    client = node.create_client(OperatingModes, '/mobile_wx250s/set_operating_modes')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Attesa del servizio /mobile_wx250s/set_operating_modes...")
    req = OperatingModes.Request()
    req.cmd_type = "group"
    req.name = "arm"              # deve corrispondere al gruppo che stai comandando
    req.mode = "position"         # modalità operativa (adatta se necessario)
    req.profile_type = "time"
    req.profile_velocity = 0
    req.profile_acceleration = 0
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info("Profili impostati correttamente.")
    else:
        node.get_logger().error("Errore nell'impostazione dei profili.")

    # Avviso che la traiettoria sta iniziando
    node.get_logger().info("Inizio Traiettoria.")
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
