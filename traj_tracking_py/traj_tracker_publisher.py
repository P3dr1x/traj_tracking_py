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
        # Periodo del timer in secondi
        self.timer_period = 0.001  # potrei metterci il primo punto della colonna time
        

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

    def _set_robot_profiles(self, profile_type: str, profile_velocity: int, profile_acceleration: int, mode: str = "position", group_name: str = "arm") -> bool:
        """
        Chiama il servizio per impostare i profili operativi del robot.
        Restituisce True in caso di successo, False altrimenti.
        """
        self.get_logger().info(f"Tentativo di impostare profili: type='{profile_type}', prof_vel={profile_velocity}, prof_accel={profile_acceleration} per gruppo '{group_name}' in '{mode}' mode.")
        
        # Assicurati che il nome del servizio sia corretto per la tua configurazione
        service_name = '/mobile_wx250s/set_operating_modes'
        client = self.create_client(OperatingModes, service_name)

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Attesa del servizio /mobile_wx250s/set_operating_modes...")

        req = OperatingModes.Request()
        req.cmd_type = "group"
        req.name = group_name
        req.mode = mode
        req.profile_type = profile_type
        req.profile_velocity = profile_velocity
        req.profile_acceleration = profile_acceleration
        
        future = client.call_async(req)
        
        # Attendi il completamento del future
        # Nota: spin_until_future_complete bloccherà l'esecuzione di questo callback
        # finché il future non sarà completo o non scadrà il timeout.
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.done():
            try:
                result = future.result() # Questo può sollevare un'eccezione se il servizio l'ha fatto
                if result is not None: # Assumendo che 'None' non sia una risposta di successo valida
                    self.get_logger().info(f"Profili impostati correttamente tramite il servizio '{service_name}'.")
                    return True
                else:
                    # Se il servizio può legittimamente restituire None per un fallimento gestito
                    self.get_logger().error(f"Errore nell'impostazione dei profili: il servizio '{service_name}' ha risposto None.")
                    return False
            except Exception as e:
                # Questo cattura le eccezioni sollevate dal servizio e memorizzate nel future
                self.get_logger().error(f"Errore nell'impostazione dei profili: eccezione dal servizio '{service_name}': {e}")
                return False
        else: # Timeout di spin_until_future_complete
            self.get_logger().error(f"Timeout durante l'attesa della risposta dal servizio '{service_name}' per impostare i profili.")
            return False



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
            if self.timer is not None:
                self.timer.cancel() # Usa cancel() per fermare il timer
                self.timer = None # Opzionale: imposta a None per indicare che è stato cancellato

            self.get_logger().info("Ripristino profili motore (vel=2000, accel=300)...")
            success_reset = self._set_robot_profiles(profile_type="time",
                                                     profile_velocity=2000,
                                                     profile_acceleration=300)
            if success_reset:
                self.get_logger().info("Profili motore ripristinati correttamente.")
            else:
                self.get_logger().warn("Fallimento nel ripristino dei profili motore.")
            
            self._perform_shutdown()

    def _perform_shutdown(self):
        """Gestisce la distruzione del nodo e lo shutdown di rclpy."""
        if hasattr(self, '_shutting_down') and self._shutting_down:
            return # Già in fase di shutdown
        self._shutting_down = True

        self.get_logger().info("Avvio procedura di chiusura del nodo...")
        # È importante distruggere il nodo prima di chiamare rclpy.shutdown()
        # per rilasciare correttamente le risorse del nodo.
        self.destroy_node() 
        
        # rclpy.shutdown() dovrebbe essere chiamato solo una volta.
        # Controlla se rclpy è ancora attivo prima di chiamare shutdown.
        if rclpy.ok():
            self.get_logger().info("Chiamata a rclpy.shutdown().")
            rclpy.shutdown()
        else:
            self.get_logger().info("rclpy è già stato spento.")
            

def main():
    rclpy.init()
    if len(sys.argv) < 2:
        print("Uso: ros2 run traj_tracking_py traj_tracker_publisher path/to/trajectory.csv")
        rclpy.try_shutdown() # Aggiungi questo
        return

    csv_file = sys.argv[1]
    node = TrajTrackerPublisher(csv_file)

    # 1. Comando "Home": invio un messaggio (congiunto a zero) per mandare il robot nella configurazione Home.
    home_msg = JointGroupCommand()
    home_msg.name = "arm"  # assicurati che corrisponda al gruppo controllato
    #home_msg.cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # configurazione Home: adatta se necessario
    home_msg.cmd = node.points[1]  # configurazione Home: adatta se necessario
    node.get_logger().info("Vado alla configurazione di partenza della traiettoria")
    node.pub.publish(home_msg)
    time.sleep(2.0)  # attesa per garantire il raggiungimento della posizione Home   

    # 2. Impostazione dei profili di velocità e accelerazione a zero
    node.get_logger().info("Impostazione profili iniziali (profile_type='time', velocità e accelerazione = 0)...")
    success_init_profile = node._set_robot_profiles(profile_type="time",
                                                    profile_velocity=0,
                                                    profile_acceleration=0)

    # Avviso che la traiettoria sta iniziando
    node.get_logger().info("Inizio Traiettoria...")
    
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Errore imprevisto durante rclpy.spin: {e}")
        node._perform_shutdown() # Tenta una chiusura pulita anche in caso di altri errori
    finally:
        # Assicurarsi che rclpy sia spento se non lo è già stato,
        # specialmente se lo spin esce in modo anomalo e _perform_shutdown non è stato chiamato.
        if rclpy.ok():
            node.get_logger().warn("rclpy.spin è terminato, ma rclpy è ancora attivo. Tentativo di shutdown finale.")
            rclpy.shutdown()

if __name__ == '__main__':
    main()
