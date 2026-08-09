#!/usr/bin/env python3
import os
import csv
import argparse

def load_data(csv_path):
    if not os.path.exists(csv_path):
        print(f"Error: File not found at {csv_path}")
        return None
    
    rows = []
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if row and len(row) > 0 and row[0] in ('PID', 'YAW'):
                rows.append(row)
    return rows

def run_correlation(rows):
    times = []
    nx = []
    ny = []
    yaw_err = []
    rc_r = []
    rc_p = []
    rc_y = []
    
    for row in rows:
        try:
            times.append(float(row[1]))
            nx.append(float(row[9]))
            ny.append(float(row[10]))
            y_err = float(row[13]) if (len(row) > 13 and row[13] != '') else 0.0
            yaw_err.append(y_err)
            rc_r.append(float(row[21]) - 1500 if (len(row) > 21 and row[21] != '') else 0.0)
            rc_p.append(float(row[27]) - 1500 if (len(row) > 27 and row[27] != '') else 0.0)
            rc_y.append(float(row[33]) - 1500 if (len(row) > 33 and row[33] != '') else 0.0)
        except (ValueError, IndexError):
            continue

    print(f"Total PID/YAW rows: {len(times)}")
    if len(times) > 1:
        dnx = [nx[i] - nx[i-1] for i in range(1, len(nx))]
        rc_r_prev = rc_r[:-1]
        
        dny = [ny[i] - ny[i-1] for i in range(1, len(ny))]
        rc_p_prev = rc_p[:-1]

        dyaw = [yaw_err[i] - yaw_err[i-1] for i in range(1, len(yaw_err))]
        rc_y_prev = rc_y[:-1]

        def corr(x, y):
            n = len(x)
            sum_x, sum_y = sum(x), sum(y)
            sum_xy = sum(x[i]*y[i] for i in range(n))
            sum_x2 = sum(x[i]**2 for i in range(n))
            sum_y2 = sum(y[i]**2 for i in range(n))
            denom = ((n * sum_x2 - sum_x**2) * (n * sum_y2 - sum_y**2)) ** 0.5
            if denom == 0: return 0
            return (n * sum_xy - sum_x * sum_y) / denom

        print(f"Roll correlation:  {corr(rc_r_prev, dnx):.6f}")
        print(f"Pitch correlation: {corr(rc_p_prev, dny):.6f}")
        print(f"Yaw correlation:   {corr(rc_y_prev, dyaw):.6f}")

def print_pitch_data(rows, start, end):
    print("Time, Err_Y, RC_P")
    total = len(rows)
    if total == 0:
        return
    start = max(0, min(start, total - 1))
    end = max(0, min(end, total))
    for i in range(start, end):
        row = rows[i]
        try:
            print(f"{row[1]}, {float(row[10]):.3f}, {row[27]}")
        except (ValueError, IndexError):
            continue

def print_roll_data(rows, start, end):
    print("Time, Err_X, RC_R")
    total = len(rows)
    if total == 0:
        return
    start = max(0, min(start, total - 1))
    end = max(0, min(end, total))
    for i in range(start, end):
        row = rows[i]
        try:
            print(f"{row[1]}, {float(row[9]):.3f}, {row[21]}")
        except (ValueError, IndexError):
            continue

def print_yaw_data(rows, start, end):
    print("Time, Yaw_Err(rad), RC_Y")
    total = len(rows)
    if total == 0:
        return
    start = max(0, min(start, total - 1))
    end = max(0, min(end, total))
    for i in range(start, end):
        row = rows[i]
        try:
            yaw_val = float(row[13]) if (len(row) > 13 and row[13] != '') else 0.0
            rc_y_val = row[33] if len(row) > 33 else ''
            print(f"{row[1]}, {yaw_val:.3f}, {rc_y_val}")
        except (ValueError, IndexError):
            continue


def analyze_vision_log(filepath):
    if not os.path.exists(filepath):
        print(f"Error: Vision log not found at {filepath}")
        return
        
    total_frames = 0
    detected_count = 0
    rejected_sum = 0
    marker_counts = {}
    
    longest_no_streak = 0
    current_no_streak = 0
    
    timestamps = []
    
    with open(filepath, 'r') as f:
        for line in f:
            if '|' not in line:
                continue
            parts = [p.strip() for p in line.split('|')]
            if len(parts) < 6:
                continue
            if parts[0] == 'TIMESTAMP' or parts[0].startswith('---'):
                continue
                
            total_frames += 1
            timestamps.append(parts[0])
            
            detected = parts[4].upper()
            if detected == 'YES':
                detected_count += 1
                if current_no_streak > longest_no_streak:
                    longest_no_streak = current_no_streak
                current_no_streak = 0
                
                m_id = parts[6]
                if m_id != '---':
                    marker_counts[m_id] = marker_counts.get(m_id, 0) + 1
            elif detected == 'NO':
                current_no_streak += 1
                
            try:
                rejected_sum += int(parts[5])
            except ValueError:
                pass
                
    if current_no_streak > longest_no_streak:
        longest_no_streak = current_no_streak
        
    print(f"Total frames processed: {total_frames}")
    if total_frames > 0:
        det_rate = (detected_count / total_frames) * 100
        print(f"Detected frames: {detected_count} ({det_rate:.2f}%)")
        print(f"Not detected frames: {total_frames - detected_count} ({100 - det_rate:.2f}%)")
        print(f"Average rejected markers/frame: {rejected_sum / total_frames:.2f}")
        print(f"Longest continuous loss of detection streak: {longest_no_streak} frames")
        if timestamps:
            print(f"Log Duration: from {timestamps[0]} to {timestamps[-1]}")
        print("Detected Marker IDs distribution:")
        for m_id, count in sorted(marker_counts.items()):
            print(f"  - Marker ID {m_id}: {count} times")

def analyze_mission_log(filepath):
    if not os.path.exists(filepath):
        print(f"Error: Mission log not found at {filepath}")
        return
        
    parameters = {}
    events = []
    
    roll_errs = []
    pitch_errs = []
    altitudes = []
    distances = []
    
    with open(filepath, 'r') as f:
        lines = f.readlines()
        
    i = 0
    n = len(lines)
    while i < n:
        line = lines[i].strip()
        if not line:
            i += 1
            continue
            
        if "LOADED PARAMETERS:" in line:
            try:
                param_part = line.split("LOADED PARAMETERS:")[1].strip()
                pairs = param_part.split(",")
                for pair in pairs:
                    k, v = pair.split("=")
                    parameters[k.strip()] = v.strip()
            except Exception:
                pass
            i += 1
            continue
            
        if "Menunggu marker ArUco terdeteksi..." in line:
            events.append((line[:10], "Waiting for ArUco marker detection"))
        elif "Mulai fase Centering presisi..." in line:
            events.append((line[:10], "Precision centering started"))
        elif any(kw in line for kw in ["Failsafe", "LAND", "abort", "hover", "override"]):
            events.append((line[:10], line[10:].strip()))
            
        if "DEBUG PID:" in line:
            try:
                i += 1
                roll_line = lines[i].strip()
                if "Roll :" in roll_line:
                    err_part = roll_line.split("Err=")[1].split("|")[0].strip()
                    roll_errs.append(abs(float(err_part)))
                    
                i += 1
                pitch_line = lines[i].strip()
                if "Pitch:" in pitch_line:
                    err_part = pitch_line.split("Err=")[1].split("|")[0].strip()
                    pitch_errs.append(abs(float(err_part)))
                    
                i += 1
                dist_line = lines[i].strip()
                if "Dist=" in dist_line:
                    dist_part = dist_line.split("Dist=")[1].split("|")[0].strip()
                    alt_part = dist_line.split("Alt=")[1].replace("m", "").strip()
                    distances.append(float(dist_part))
                    altitudes.append(float(alt_part))
            except (IndexError, ValueError):
                pass
        i += 1
        
    print("\n--- Mission Configuration ---")
    if parameters:
        for k, v in parameters.items():
            print(f"  {k}: {v}")
    else:
        print("  No loaded parameters found in log.")
        
    print("\n--- Event Timeline ---")
    if events:
        for ts, ev in events:
            print(f"  {ts} {ev}")
    else:
        print("  No major events found in log.")
        
    print("\n--- PID Controller Metrics ---")
    if roll_errs:
        print(f"  Total PID Updates: {len(roll_errs)}")
        print(f"  Average Roll absolute error: {sum(roll_errs)/len(roll_errs):.4f}")
        print(f"  Max Roll absolute error: {max(roll_errs):.4f}")
        print(f"  Average Pitch absolute error: {sum(pitch_errs)/len(pitch_errs):.4f}")
        print(f"  Max Pitch absolute error: {max(pitch_errs):.4f}")
    else:
        print("  No PID metrics data parsed.")
        
    print("\n--- Physical Tracking Metrics ---")
    if distances:
        print(f"  Average Distance from center: {sum(distances)/len(distances):.4f} m")
        print(f"  Max Distance from center: {max(distances):.4f} m")
        print(f"  Average Altitude: {sum(altitudes)/len(altitudes):.2f} m")
        print(f"  Altitude range: {min(altitudes):.2f}m to {max(altitudes):.2f}m")
    else:
        print("  No distance/altitude tracking data parsed.")

def main():
    parser = argparse.ArgumentParser(description="Troubleshooting & analysis tool for VTOL Centering, Vision, and PID logs")
    parser.add_argument('--csv', '-f', type=str, default=None,
                        help="Path to the centering CSV file")
    parser.add_argument('--vision-log', '-vl', type=str, default=None,
                        help="Path to the aruco vision log file")
    parser.add_argument('--mission-log', '-ml', type=str, default=None,
                        help="Path to the mission centering log file")
    parser.add_argument('--mode', '-m', type=str, choices=['correlation', 'pitch', 'roll', 'yaw', 'vision', 'mission', 'all'], default='all',
                        help="Analysis mode: correlation, pitch, roll, yaw, vision, mission, or all (default: all)")
    parser.add_argument('--start', '-s', type=int, default=100,
                        help="Start index for printing detailed data (default: 100)")
    parser.add_argument('--end', '-e', type=int, default=120,
                        help="End index for printing detailed data (default: 120)")
    
    args = parser.parse_args()
    
    # Resolve CSV path
    csv_path = args.csv
    if not csv_path:
        if os.path.exists('centering_data.csv'):
            csv_path = 'centering_data.csv'
        else:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            script_dir_csv = os.path.join(script_dir, 'centering_data.csv')
            if os.path.exists(script_dir_csv):
                csv_path = script_dir_csv
            elif os.path.exists(os.path.join('workspace', 'centering_data.csv')):
                csv_path = os.path.join('workspace', 'centering_data.csv')
            else:
                csv_path = script_dir_csv

    # Resolve Vision Log path
    vision_path = args.vision_log
    if not vision_path:
        if os.path.exists('aruco_vision.log'):
            vision_path = 'aruco_vision.log'
        else:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            script_dir_vl = os.path.join(script_dir, 'aruco_vision.log')
            if os.path.exists(script_dir_vl):
                vision_path = script_dir_vl
            elif os.path.exists(os.path.join('workspace', 'aruco_vision.log')):
                vision_path = os.path.join('workspace', 'aruco_vision.log')
            else:
                vision_path = script_dir_vl

    # Resolve Mission Log path
    mission_path = args.mission_log
    if not mission_path:
        if os.path.exists('mission_centering.log'):
            mission_path = 'mission_centering.log'
        else:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            script_dir_ml = os.path.join(script_dir, 'mission_centering.log')
            if os.path.exists(script_dir_ml):
                mission_path = script_dir_ml
            elif os.path.exists(os.path.join('workspace', 'mission_centering.log')):
                mission_path = os.path.join('workspace', 'mission_centering.log')
            else:
                mission_path = script_dir_ml

    # Execute selected modes
    if args.mode in ('correlation', 'pitch', 'roll', 'yaw', 'all'):
        print(f"=== Centering Data Analysis ({csv_path}) ===")
        rows = load_data(csv_path)
        if rows:
            if args.mode in ('correlation', 'all'):
                print("\n--- Correlation Analysis ---")
                run_correlation(rows)
                
            if args.mode in ('pitch', 'all'):
                print(f"\n--- Pitch Data (Indices {args.start} to {args.end}) ---")
                print_pitch_data(rows, args.start, args.end)
                
            if args.mode in ('roll', 'all'):
                print(f"\n--- Roll Data (Indices {args.start} to {args.end}) ---")
                print_roll_data(rows, args.start, args.end)

            if args.mode in ('yaw', 'all'):
                print(f"\n--- Yaw Data (Indices {args.start} to {args.end}) ---")
                print_yaw_data(rows, args.start, args.end)
        else:
            print("Failed to load centering data CSV.")

    if args.mode in ('vision', 'all'):
        print(f"\n=== Aruco Vision Log Analysis ({vision_path}) ===")
        analyze_vision_log(vision_path)

    if args.mode in ('mission', 'all'):
        print(f"\n=== Mission Centering Log Analysis ({mission_path}) ===")
        analyze_mission_log(mission_path)

if __name__ == '__main__':
    main()
