import os
from datetime import datetime
import pandas as pd
import matplotlib.pyplot as plt
import csv

log_filename = "acc_log.csv"

def log_metrics(ego_speed, lead_speed, throttle, brake):
    fieldnames = ["timestamp", "ego_speed", "lead_speed", "throttle", "brake"]

    try:
        write_header = True
        if os.path.exists(log_filename):
            with open(log_filename, "r") as check_file:
                first_line = check_file.readline().strip()
                write_header = first_line != ",".join(fieldnames)

        with open(log_filename, "a", newline='') as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            if write_header:
                writer.writeheader()
            writer.writerow({
                "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "ego_speed": ego_speed,
                "lead_speed": lead_speed,
                "throttle": throttle,
                "brake": brake
            })
    except PermissionError:
        print("⚠️ Unable to write log (acc_log.csv is in use by another program)")

def plot_acc_log():
    if not os.path.exists(log_filename):
        print("⚠️ There is no log file for plot.")
        return

    try:
        df = pd.read_csv(log_filename)

        required_columns = {"timestamp", "ego_speed", "lead_speed", "throttle", "brake"}
        if not required_columns.issubset(set(df.columns)):
            print("❌ The log file is incomplete or the header is incorrect. Please delete acc_log.csv and rerun.")
            return

        fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
        fig.suptitle("ACC Log Metrics (No Distance)", fontsize=16)

        axs[0].plot(df["ego_speed"], label="Ego Speed (m/s)")
        axs[0].plot(df["lead_speed"], label="Lead Speed (m/s)", linestyle="--")
        axs[0].set_ylabel("Speed (m/s)")
        axs[0].legend()
        axs[0].grid(True)

        axs[1].plot(df["throttle"], label="Throttle", color="green")
        axs[1].plot(df["brake"], label="Brake", color="red")
        axs[1].set_ylabel("Control")
        axs[1].legend()
        axs[1].grid(True)

        plt.xlabel("Frame / Time Step")
        plt.tight_layout()
        plt.subplots_adjust(top=0.90)

        plt.savefig("acc_log_plot.png")
        plt.savefig("acc_log_plot.pdf")
        print("✅ บันทึกกราฟเป็น acc_log_plot.png และ .pdf แล้ว")

        plt.show()

    except Exception as e:
        print(f"❌ An error occurred while plotting log.: {e}")
