import tkinter as tk
from tkinter import ttk, messagebox
import asyncio
from bleak import BleakClient, BleakScanner
import serial
import serial.tools.list_ports
import threading
import csv
from datetime import datetime
import re  # Do parsowania danych z portu szeregowego

# UUIDs z kodu Arduino
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHARACTERISTIC_UUID_TX = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
CHARACTERISTIC_UUID_RX = "faaa1ce4-eca1-4a11-b9e2-715d451b5d14"

class ECGApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Monitor EKG")
        self.client = None  # Obiekt BLE
        self.ser = None     # Obiekt portu szeregowego
        self.data_buffer = []  # Bufor na dane z portu szeregowego
        self.loop = asyncio.new_event_loop()  # Nowa pętla asyncio
        asyncio.set_event_loop(self.loop)
        self.serial_thread = None
        self.serial_connected = False  # Flaga stanu połączenia szeregowego

        # Elementy GUI
        self.status_label = tk.Label(root, text="Status: Rozłączono")
        self.status_label.pack(pady=5)

        self.connect_ble_button = tk.Button(root, text="Połącz z ESP32", command=self.connect_ble)
        self.connect_ble_button.pack(pady=5)

        self.port_label = tk.Label(root, text="Wybierz port szeregowy:")
        self.port_label.pack(pady=5)

        self.port_combobox = ttk.Combobox(root, values=self.get_serial_ports())
        self.port_combobox.pack(pady=5)

        self.refresh_ports_button = tk.Button(root, text="Odśwież porty", command=self.refresh_ports)
        self.refresh_ports_button.pack(pady=5)

        self.connect_serial_button = tk.Button(root, text="Połącz z portem", command=self.connect_serial)
        self.connect_serial_button.pack(pady=5)

        self.serial_data_text = tk.Text(root, height=10, width=50)
        self.serial_data_text.pack(pady=5)

        self.save_csv_button = tk.Button(root, text="Zapisz do CSV", command=self.save_to_csv)
        self.save_csv_button.pack(pady=5)

        self.lead1_button = tk.Button(root, text="Wyświetl pierwsze odprowadzenie", command=lambda: self.send_command('1'))
        self.lead1_button.pack(pady=2)

        self.lead2_button = tk.Button(root, text="Wyświetl drugie odprowadzenie", command=lambda: self.send_command('2'))
        self.lead2_button.pack(pady=2)

        self.lead3_button = tk.Button(root, text="Wyświetl trzecie odprowadzenie", command=lambda: self.send_command('3'))
        self.lead3_button.pack(pady=2)

        self.all_leads_button = tk.Button(root, text="Wyświetl wszystkie odprowadzenia", command=lambda: self.send_command('4'))
        self.all_leads_button.pack(pady=2)

        self.enable_filters_button = tk.Button(root, text="Wyłącz wszystkie filtry", command=lambda: self.send_command('6'))
        self.enable_filters_button.pack(pady=2)

        self.disable_filters_button = tk.Button(root, text="Włącz wszystkie filtry", command=lambda: self.send_command('7'))
        self.disable_filters_button.pack(pady=2)

        self.asyncio_thread = threading.Thread(target=self.run_asyncio_loop, daemon=True)
        self.asyncio_thread.start()

    def run_asyncio_loop(self):
        """Uruchamia pętlę asyncio w osobnym wątku."""
        self.loop.run_forever()

    def get_serial_ports(self):
        """Zwraca listę dostępnych portów szeregowych."""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        if not ports:
            messagebox.showwarning("Brak portów", "Nie wykryto portów szeregowych. Sprawdź uprawnienia lub podłączenie.")
        return ports

    def refresh_ports(self):
        """Odświeża listę portów w comboboxie."""
        self.port_combobox['values'] = self.get_serial_ports()
        if '/dev/ttyACM0' in self.get_serial_ports():
            self.port_combobox.set('/dev/ttyACM0')

    def connect_ble(self):
        """Inicjuje połączenie BLE."""
        self.status_label.config(text="Status: Łączenie...")
        threading.Thread(target=self._connect_ble_thread, daemon=True).start()

    def _connect_ble_thread(self):
        """Uruchamia asynchroniczne połączenie BLE w pętli asyncio."""
        asyncio.run_coroutine_threadsafe(self._connect_ble_with_timeout(), self.loop)

    async def _connect_ble_with_timeout(self):
        """Próbuje połączyć się z ESP32 z timeoutem 10 sekund."""
        try:
            await asyncio.wait_for(self._connect_ble(), timeout=10)
        except asyncio.TimeoutError:
            self.root.after(0, lambda: self.status_label.config(text="Status: Błąd: Timeout"))
            self.root.after(0, lambda: messagebox.showerror("Błąd", "Przekroczono czas oczekiwania. Spróbuj ponownie."))
        except Exception as e:
            self.root.after(0, lambda: self.status_label.config(text=f"Status: Błąd: {e}"))
            self.root.after(0, lambda: messagebox.showerror("Błąd", f"Nie udało się połączyć: {e}"))

    async def _connect_ble(self):
        """Łączy się z urządzeniem ESP32 przez BLE."""
        devices = await BleakScanner.discover(timeout=5.0)
        for device in devices:
            print(f"Znaleziono: {device.name}, {device.address}")
            if device.name == "ESP32":
                self.client = BleakClient(device.address)
                await self.client.connect()
                await self.client.start_notify(CHARACTERISTIC_UUID_TX, self.notification_handler)
                self.root.after(0, lambda: self.status_label.config(text="Status: Połączono"))
                return
        self.root.after(0, lambda: self.status_label.config(text="Status: Nie znaleziono ESP32"))
        self.root.after(0, lambda: messagebox.showerror("Błąd", "Nie znaleziono urządzenia ESP32."))

    def notification_handler(self, sender, data):
        """Obsługuje notyfikacje z BLE."""
        try:
            decoded = data.decode()
            values = decoded.split(',')
            lead1 = int(values[0].strip())
            lead2 = int(values[1].strip())
            lead3 = int(values[2].strip())
            print(f"BLE - Lead1: {lead1}, Lead2: {lead2}, Lead3: {lead3}")
        except Exception as e:
            print(f"Błąd parsowania danych BLE: {e}")

    def connect_serial(self):
        """Łączy się z wybranym portem szeregowym."""
        port = self.port_combobox.get()
        if port:
            try:
                self.ser = serial.Serial(port, 115200, timeout=1)
                self.serial_connected = True
                self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
                self.serial_thread.start()
                messagebox.showinfo("Info", f"Połączono z portem: {port}")
            except Exception as e:
                messagebox.showerror("Błąd", f"Nie udało się połączyć z portem: {e}")
        else:
            messagebox.showerror("Błąd", "Wybierz port szeregowy!")

    def read_serial(self):
        """Odczytuje dane z portu szeregowego i zapisuje je do bufora."""
        while self.serial_connected:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode().strip()
                    self.root.after(0, lambda: self.serial_data_text.insert(tk.END, f"{line}\n"))
                    self.root.after(0, lambda: self.serial_data_text.see(tk.END))
                    # Parsowanie danych w formacie "Serial: Lead1: x, Lead2: y, Lead3: z"
                    match = re.search(r'Lead1:\s*(\d+),\s*Lead2:\s*(\d+),\s*Lead3:\s*(\d+)', line)
                    if match:
                        lead1, lead2, lead3 = map(int, match.groups())
                        self.data_buffer.append((lead1, lead2, lead3))
            except (OSError, serial.SerialException) as e:
                self.serial_connected = False
                self.root.after(0, lambda: messagebox.showerror("Błąd", "Port szeregowy jest niedostępny. Sprawdź połączenie."))
                break
            except Exception as e:
                print(f"Błąd w read_serial: {e}")

    def send_command(self, command):
        """Wysyła komendę przez BLE."""
        if self.client and self.client.is_connected:
            asyncio.run_coroutine_threadsafe(self._send_command(command), self.loop)
        else:
            messagebox.showerror("Błąd", "Brak połączenia z ESP32!")

    async def _send_command(self, command):
        """Asynchroniczne wysyłanie komendy przez BLE."""
        try:
            await self.client.write_gatt_char(CHARACTERISTIC_UUID_RX, command.encode())
            print(f"Wysłano komendę: {command}")
        except Exception as e:
            print(f"Błąd wysyłania komendy: {e}")

    def save_to_csv(self):
        """Zapisuje dane z bufora do pliku CSV."""
        if not self.data_buffer:
            messagebox.showinfo("Info", "Brak danych do zapisania!")
            return
        filename = f"ecg_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        try:
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['lead1', 'lead2', 'lead3'])
                writer.writerows(self.data_buffer)
            messagebox.showinfo("Info", f"Dane zapisane do {filename}")
            self.data_buffer.clear()
        except Exception as e:
            messagebox.showerror("Błąd", f"Nie udało się zapisać pliku: {e}")

if __name__ == "__main__":
    root = tk.Tk()
    app = ECGApp(root)
    root.mainloop()