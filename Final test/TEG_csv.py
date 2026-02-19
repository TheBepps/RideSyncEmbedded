import os
import time
from datetime import datetime
import os


# Parametri
file_path = "/home/user/Desktop/temp2.txt"   # il tuo file CSV/TXT
write_time_estimate = 2.5  # tempo stimato per scrittura + invio
click_x, click_y = 190, 598  # posizione del click
skip_entries = 15  # prendi una riga ogni 5

# Funzione per convertire "HH:MM:SS.microsec" in datetime
def parse_time(t):
    return datetime.strptime(t, "%H:%M:%S.%f")

# Legge il file CSV/TXT
raw_data = []
with open(file_path, "r") as f:
    lines = f.readlines()
    for idx, line in enumerate(lines):
        if idx % skip_entries != 0:  # salta righe secondo skip_entries
            continue
        line = line.strip()
        if line:
            raw_data.append(line)

# Convertiamo in lista di (time, value)
entries = []
for row in raw_data:
    date, t, val = row.split(",")
    entries.append((parse_time(t), (float(val)+10.0)))

# Esegui ogni comando con timing corretto
for i in range(len(entries)):
    current_time, value = entries[i]

    # 1. CLICK sulla posizione
    os.system(f"xdotool mousemove {click_x} {click_y} click 1")
    time.sleep(write_time_estimate)  # piccola pausa dopo il click

    # 2. CANCELLA contenuto precedente
    os.system("xdotool key Ctrl+a")  # seleziona tutto
    time.sleep(write_time_estimate/2)
    os.system("xdotool key Ctrl+a")  # seleziona tutto (come nel tuo script)
    time.sleep(write_time_estimate/2)
    os.system("xdotool key BackSpace")  # cancella
    time.sleep(write_time_estimate/2)
    

    # 3. SCRIVI il nuovo valore
    os.system(f"xdotool mousemove {click_x} {click_y} click 1")
    os.system(f"xdotool type '>setpoint {value}'")
    time.sleep(3*write_time_estimate)
    os.system("xdotool key Return")  # premi invio

    # 4. ATTESA fino al prossimo timestamp, considerando il tempo di scrittura
    if i < len(entries) - 1:
        next_time = entries[i+1][0]
        delta = (next_time - current_time).total_seconds() - 5.5*write_time_estimate
        if delta < 0:
            delta = 0
        time.sleep(delta)
