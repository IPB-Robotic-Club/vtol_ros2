"""
servo_drop.py — Node ROS2 untuk mengendalikan servo payload drop melalui MAVROS.

Mekanisme:
  - Servo terhubung ke Pixhawk AUX 5 (SERVO13) yang dipetakan ke RCIN9
  - Dikontrol via RC Override channel 9 (index array MAVROS: 8)
  - Pixhawk akan merutekan override RC ke output servo fisik

Mode operasi:
  1. Terminal Interaktif — Sub-menu [L]ow | [M]id | [H]igh | [D]rop | [Q]uit
  2. ROS2 Service       — /vtol/payload/drop (std_srvs/srv/Trigger)
  3. ROS2 Topic         — /vtol/payload/command (std_msgs/String): "low","mid","high","drop","hold"
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vtol_control.config_reader import get_payload_config
import time
import threading


class ServoDropNode(Node):
    """
    Node untuk mengendalikan servo payload drop melalui MAVROS RC Override.
    """

    def __init__(self):
        super().__init__('servo_drop_node')

        # ── Load Konfigurasi dari YAML ───────────────────────────────────────
        cfg = get_payload_config()
        self.servo_channel  = int(cfg['servo_channel'])    # RC Channel (9)
        self.pwm_hold       = int(cfg['pwm_hold'])         # posisi TUTUP
        self.pwm_drop       = int(cfg['pwm_drop'])         # posisi BUKA
        self.pwm_mid        = int(cfg['pwm_mid'])          # posisi TENGAH
        self.drop_duration  = float(cfg['drop_duration'])  # durasi drop dalam detik

        # Index array RC (0-based): channel 9 → index 8
        self._rc_index = self.servo_channel - 1

        # State internal
        self._is_dropping = False
        self._drop_timer  = None
        self._lock        = threading.Lock()

        # ── Publisher RC Override ────────────────────────────────────────────
        self.rc_pub = self.create_publisher(
            OverrideRCIn,
            '/mavros/rc/override',
            10
        )

        # ── ROS2 Service: /vtol/payload/drop ────────────────────────────────
        self.drop_service = self.create_service(
            Trigger,
            '/vtol/payload/drop',
            self._handle_drop_service
        )

        # ── ROS2 Topic: /vtol/payload/command ───────────────────────────────
        self.cmd_sub = self.create_subscription(
            String,
            '/vtol/payload/command',
            self._handle_command_topic,
            10
        )

        self.get_logger().info(
            f"[ServoDropNode] Siap (Metode: RC Override). RC Channel={self.servo_channel} | "
            f"HOLD={self.pwm_hold}µs | DROP={self.pwm_drop}µs | MID={self.pwm_mid}µs | "
            f"Drop Duration={self.drop_duration}s"
        )

        # Set servo ke posisi HOLD saat node pertama kali dimulai
        # Berikan sedikit waktu agar publisher siap
        time.sleep(0.5)
        self._send_pwm(self.pwm_hold)
        self.get_logger().info(
            f"[ServoDropNode] Servo diinisialisasi ke posisi HOLD ({self.pwm_hold}µs)"
        )

    # ── Internal: Kirim nilai PWM ke servo melalui RC Override ──────────────

    def _send_pwm(self, pwm_value: int):
        """Mengirim perintah PWM ke servo melalui topik /mavros/rc/override."""
        msg = OverrideRCIn()
        # Inisialisasi semua channel ke 0 (tidak override channel lain)
        msg.channels = [0] * 18
        # Set hanya channel servo payload
        msg.channels[self._rc_index] = pwm_value
        self.rc_pub.publish(msg)
        self.get_logger().info(
            f"[ServoDropNode] → Override RC CH{self.servo_channel} ke {pwm_value}µs"
        )

    def _release_rc_override(self):
        """Melepas override RC untuk channel servo (kembalikan ke 0 = tidak override)."""
        msg = OverrideRCIn()
        msg.channels = [0] * 18
        self.rc_pub.publish(msg)
        self.get_logger().info(
            f"[ServoDropNode] RC override channel {self.servo_channel} dilepas (nilai=0)."
        )

    # ── Aksi Servo ──────────────────────────────────────────────────────────

    def cmd_hold(self):
        """Pindahkan servo ke posisi TUTUP (HOLD) — payload ditahan."""
        with self._lock:
            if self._drop_timer and self._drop_timer.is_alive():
                self._drop_timer.cancel()
                self._drop_timer = None
            self._is_dropping = False
        self._send_pwm(self.pwm_hold)

    def cmd_mid(self):
        """Pindahkan servo ke posisi TENGAH (MID)."""
        with self._lock:
            if self._drop_timer and self._drop_timer.is_alive():
                self._drop_timer.cancel()
                self._drop_timer = None
            self._is_dropping = False
        self._send_pwm(self.pwm_mid)

    def cmd_high(self):
        """Pindahkan servo ke posisi HIGH (sama dengan DROP, tanpa auto-reset)."""
        with self._lock:
            if self._drop_timer and self._drop_timer.is_alive():
                self._drop_timer.cancel()
                self._drop_timer = None
            self._is_dropping = False
        self._send_pwm(self.pwm_drop)

    def cmd_drop(self) -> bool:
        """
        Jalankan sekuens drop:
        1. Buka servo ke posisi DROP (pwm_drop)
        2. Tunggu drop_duration detik
        3. Tutup kembali ke posisi HOLD (pwm_hold) secara otomatis
        """
        with self._lock:
            if self._is_dropping:
                self.get_logger().warn("[ServoDropNode] Drop sedang berjalan! Abaikan.")
                return False
            self._is_dropping = True

        self.get_logger().info(
            f"[ServoDropNode] DROP! Membuka servo ke {self.pwm_drop}µs selama {self.drop_duration}s..."
        )
        self._send_pwm(self.pwm_drop)

        # Timer untuk auto-reset ke HOLD setelah drop_duration selesai
        def _auto_reset():
            self.get_logger().info(
                f"[ServoDropNode] Auto-reset: Menutup servo kembali ke HOLD ({self.pwm_hold}µs)"
            )
            self._send_pwm(self.pwm_hold)
            with self._lock:
                self._is_dropping = False
                self._drop_timer = None

        with self._lock:
            self._drop_timer = threading.Timer(self.drop_duration, _auto_reset)
            self._drop_timer.daemon = True
            self._drop_timer.start()

        return True

    # ── Handler Service /vtol/payload/drop ──────────────────────────────────

    def _handle_drop_service(self, request, response):
        self.get_logger().info("[ServoDropNode] Service /vtol/payload/drop dipanggil!")
        success = self.cmd_drop()
        response.success = success
        response.message = (
            f"Payload drop berhasil! Kembali HOLD dalam {self.drop_duration}s."
            if success else
            "Gagal: drop masih berjalan."
        )
        return response

    # ── Handler Topic /vtol/payload/command ─────────────────────────────────

    def _handle_command_topic(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f"[ServoDropNode] Perintah topic: '{cmd}'")
        if cmd in ('low', 'hold'):
            self.cmd_hold()
        elif cmd == 'mid':
            self.cmd_mid()
        elif cmd == 'high':
            self.cmd_high()
        elif cmd == 'drop':
            self.cmd_drop()
        else:
            self.get_logger().warn(f"[ServoDropNode] Perintah tidak dikenal: '{cmd}'")

    # ── Cleanup ─────────────────────────────────────────────────────────────

    def destroy_node(self):
        self.get_logger().info("[ServoDropNode] Shutdown: Mengembalikan servo ke HOLD...")
        with self._lock:
            if self._drop_timer and self._drop_timer.is_alive():
                self._drop_timer.cancel()
        self._send_pwm(self.pwm_hold)
        time.sleep(0.3)
        self._release_rc_override()
        super().destroy_node()


# ── Mode Terminal Interaktif ─────────────────────────────────────────────────

def _print_servo_menu(cfg: dict):
    ch  = cfg['servo_channel']
    low = cfg['pwm_hold']
    mid = cfg['pwm_mid']
    hi  = cfg['pwm_drop']
    dur = cfg['drop_duration']
    print()
    print("╔══════════════════════════════════════════════╗")
    print("║        VTOL — TES SERVO PAYLOAD DROP         ║")
    print("╠══════════════════════════════════════════════╣")
    print(f"║  RC Override CH: {ch} (Mengendalikan RCIN{ch})       ║")
    print(f"║  PWM LOW/HOLD  : {low} µs  (payload ditahan)  ║")
    print(f"║  PWM MID       : {mid} µs  (posisi tengah)    ║")
    print(f"║  PWM HIGH/DROP : {hi}  µs  (payload dilepas) ║")
    print(f"║  Drop Duration : {dur}s  (lalu kembali HOLD) ║")
    print("╠══════════════════════════════════════════════╣")
    print("║  [L] Low  / Hold  — Servo ke posisi TUTUP   ║")
    print("║  [M] Mid          — Servo ke posisi TENGAH  ║")
    print("║  [H] High         — Servo ke posisi BUKA    ║")
    print("║  [D] Drop         — DROP + auto-reset HOLD  ║")
    print("║  [Q] Quit         — Keluar & reset servo    ║")
    print("╚══════════════════════════════════════════════╝")


def run_interactive_mode(node: ServoDropNode, cfg: dict):
    """Menjalankan loop terminal interaktif untuk tes manual servo."""
    _print_servo_menu(cfg)
    print("\nKetik perintah lalu tekan Enter:")

    while rclpy.ok():
        try:
            choice = input("  Servo >> ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            print("\nKeluar dari mode tes servo...")
            break

        if choice in ('l', 'low', 'hold'):
            node.cmd_hold()
            print(f"  ✓ Servo → LOW/HOLD ({node.pwm_hold}µs)")
        elif choice in ('m', 'mid'):
            node.cmd_mid()
            print(f"  ✓ Servo → MID ({node.pwm_mid}µs)")
        elif choice in ('h', 'high'):
            node.cmd_high()
            print(f"  ✓ Servo → HIGH ({node.pwm_drop}µs)")
        elif choice in ('d', 'drop'):
            print(f"  ⟳ DROP... (kembali HOLD dalam {node.drop_duration}s)")
            node.cmd_drop()
        elif choice in ('q', 'quit', 'exit'):
            print("  Keluar dari tes servo.")
            break
        elif choice == '':
            continue
        else:
            print(f"  ✗ Tidak valid: '{choice}'. Gunakan L/M/H/D/Q")


# ── Entry Point ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ServoDropNode()
    cfg  = get_payload_config()

    # Jalankan ROS2 spin di background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        run_interactive_mode(node, cfg)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
