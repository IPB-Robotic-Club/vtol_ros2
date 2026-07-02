import sys
import subprocess
import time

def print_menu():
    print("\n" + "="*42)
    print("         VTOL AUTONOMOUS LAUNCHER         ")
    print("="*42)
    print(" 1. Tampilkan HUD Status Drone (vehicle_status)")
    print(" 2. Jalankan Uji Coba Arming (test_arm)")
    print(" 3. Jalankan Misi Hover (mission_hover)")
    print(" 4. Jalankan Misi Maneuver (mission_maneuver)")
    print(" 5. Jalankan Misi Centering (mission_centering)")
    print(" 6. Keluar")
    print("="*42)

def main():
    while True:
        print_menu()
        try:
            choice = input("Pilih opsi (1-6): ").strip()
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        
        if choice == '1':
            print("\nMeluncurkan HUD Status... Tekan Ctrl+C untuk kembali ke menu.\n")
            time.sleep(1.0)
            try:
                # Run the ROS2 HUD viewer
                subprocess.run(["ros2", "run", "vtol_control", "vehicle_status"])
            except KeyboardInterrupt:
                pass
        elif choice == '2':
            print("\nMeluncurkan Uji Coba Arming...\n")
            time.sleep(1.0)
            try:
                # Run the ROS2 arming test
                subprocess.run(["ros2", "run", "vtol_control", "test_arm"])
            except KeyboardInterrupt:
                pass
        elif choice == '3':
            print("\nMeluncurkan Misi Hover...\n")
            time.sleep(1.0)
            try:
                # Run the ROS2 hover mission
                subprocess.run(["ros2", "run", "vtol_control", "mission_hover"])
            except KeyboardInterrupt:
                pass
        elif choice == '4':
            print("\nMeluncurkan Misi Maneuver...\n")
            time.sleep(1.0)
            try:
                # Run the ROS2 maneuver mission
                subprocess.run(["ros2", "run", "vtol_control", "mission_maneuver"])
            except KeyboardInterrupt:
                pass
        elif choice == '5':
            print("\nMeluncurkan Misi Centering...\n")
            time.sleep(1.0)
            try:
                # Run the ROS2 centering mission
                subprocess.run(["ros2", "run", "vtol_control", "mission_centering"])
            except KeyboardInterrupt:
                pass
        elif choice == '6':
            print("\nKeluar dari menu launcher.")
            break
        else:
            print("\nPilihan tidak valid. Silakan masukkan angka 1-6.")

if __name__ == '__main__':
    main()
