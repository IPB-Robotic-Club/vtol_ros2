import sys
import subprocess
import time

def print_menu():
    print("\n" + "="*50)
    print("           VTOL AUTONOMOUS LAUNCHER           ")
    print("="*50)
    print(" 1. Tampilkan HUD Status Drone (vehicle_status)")
    print(" 2. Jalankan Uji Coba Arming (test_arm)")
    print(" 3. Jalankan Uji Coba Takeoff Only (test_takeoff)")
    print(" 4. Jalankan Misi Hover 5 Detik (mission_hover)")
    print(" 5. Jalankan Misi Maneuver (mission_maneuver)")
    print(" 6. Jalankan Misi Centering (mission_centering)")
    print(" 7. Debug ArUco Dry-Run / Pre-Flight (debug_aruco)")
    print(" 8. Jalankan Vision Receiver Standalone (aruco_receiver)")
    print(" 9. Jalankan Uji Coba Servo CH9 (mission_servo)")
    print(" 10. Keluar")
    print("="*50)

def main():
    while True:
        print_menu()
        try:
            choice = input("Pilih opsi (1-10): ").strip()
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        
        if choice == '1':
            print("\nMeluncurkan HUD Status... Tekan Ctrl+C untuk kembali ke menu.\n")
            time.sleep(1.0)
            try:
                subprocess.run(["ros2", "run", "vtol_control", "vehicle_status"])
            except KeyboardInterrupt:
                pass
        elif choice == '2':
            print("\nMeluncurkan Uji Coba Arming...\n")
            time.sleep(1.0)
            try:
                subprocess.run(["ros2", "run", "vtol_control", "test_arm"])
            except KeyboardInterrupt:
                pass
        elif choice == '3':
            print("\nMeluncurkan Uji Coba Takeoff Only...")
            print("Drone akan takeoff dan hover di udara sampai Anda menekan Ctrl+C (LAND).\n")
            time.sleep(1.0)
            try:
                subprocess.run(["ros2", "run", "vtol_control", "test_takeoff"])
            except KeyboardInterrupt:
                pass
        elif choice == '4':
            print("\nMeluncurkan Misi Hover 5 Detik...\n")
            time.sleep(1.0)
            try:
                subprocess.run(["ros2", "run", "vtol_control", "mission_hover"])
            except KeyboardInterrupt:
                pass
        elif choice == '5':
            print("\nMeluncurkan Misi Maneuver...\n")
            time.sleep(1.0)
            try:
                subprocess.run(["ros2", "run", "vtol_control", "mission_maneuver"])
            except KeyboardInterrupt:
                pass
        elif choice == '6':
            print("\nMeluncurkan Misi Centering...\n")
            time.sleep(1.0)
            try:
                subprocess.run(["ros2", "run", "vtol_control", "mission_centering"])
            except KeyboardInterrupt:
                pass
        elif choice == '7':
            print("\nMeluncurkan Debug ArUco Dry-Run...")
            print("Drone TIDAK akan terbang. Menampilkan telemetry & kalkulasi PID ArUco.")
            print("Tekan Ctrl+C untuk kembali ke menu.\n")
            time.sleep(1.0)
            try:
                subprocess.run(["ros2", "run", "vtol_control", "debug_aruco"])
            except KeyboardInterrupt:
                pass
        elif choice == '8':
            print("\nMeluncurkan Vision ArUco Receiver Standalone...")
            print("Tekan Ctrl+C untuk kembali ke menu.\n")
            time.sleep(1.0)
            try:
                subprocess.run(["ros2", "run", "vtol_vision", "aruco_receiver"])
            except KeyboardInterrupt:
                pass
        elif choice == '9':
            print("\nMeluncurkan Uji Coba Servo CH9...")
            print("Tekan Ctrl+C untuk kembali ke menu.\n")
            time.sleep(1.0)
            try:
                subprocess.run(["ros2", "run", "vtol_control", "mission_servo"])
            except KeyboardInterrupt:
                pass
        elif choice == '10':
            print("\nKeluar dari menu launcher.")
            break
        else:
            print("\nPilihan tidak valid. Silakan masukkan angka 1-10.")

if __name__ == '__main__':
    main()
