import sys
import subprocess
import time

def print_menu():
    print("\n" + "="*42)
    print("         VTOL AUTONOMOUS LAUNCHER         ")
    print("="*42)
    print(" 1. Tampilkan HUD Status Drone (vehicle_status)")
    print(" 2. Jalankan Uji Coba Arming (test_arm)")
    print(" 3. Keluar")
    print("="*42)

def main():
    while True:
        print_menu()
        try:
            choice = input("Pilih opsi (1-3): ").strip()
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
            print("\nKeluar dari menu launcher.")
            break
        else:
            print("\nPilihan tidak valid. Silakan masukkan angka 1-3.")

if __name__ == '__main__':
    main()
