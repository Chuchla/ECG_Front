import asyncio
from bleak import BleakClient, BleakScanner
import serial
import serial.tools.list_ports
import threading

# UUIDs z kodu Arduino
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHARACTERISTIC_UUID_TX = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
CHARACTERISTIC_UUID_RX = "faaa1ce4-eca1-4a11-b9e2-715d451b5d14"

# Globalne zmienne
client = None
ser = None
print_serial = False  # Domyślnie wyświetlanie danych z portu szeregowego włączone

# Obsługa danych z BLE (TX)
def notification_handler(sender, data):
    try:
        decoded = data.decode()
        values = decoded.split(',')
        lead1 = int(values[0].strip())
        lead2 = int(values[1].strip())
        lead3 = int(values[2].strip())
        print(f"BLE - Lead1: {lead1}, Lead2: {lead2}, Lead3: {lead3}")
    except Exception as e:
        print(f"Błąd parsowania danych BLE: {e}")

# Połączenie z BLE
async def connect_ble():
    global client
    devices = await BleakScanner.discover()
    for d in devices:
        if d.name == "ESP32":
            client = BleakClient(d.address)
            await client.connect()
            await client.start_notify(CHARACTERISTIC_UUID_TX, notification_handler)
            print("Połączono z ESP32 przez BLE")
            break

# Wysłanie komendy na RX
async def send_command(command):
    global client
    if client and client.is_connected:
        try:
            await client.write_gatt_char(CHARACTERISTIC_UUID_RX, command.encode())
            print(f"Wysłano komendę: {command}")
        except Exception as e:
            print(f"Błąd wysyłania komendy: {e}")
    else:
        print("Brak połączenia z urządzeniem BLE")

# Odczyt z portu szeregowego
def read_serial():
    global ser, print_serial
    while True:
        if ser and ser.in_waiting > 0:
            line = ser.readline().decode().strip()
            if print_serial:  # Wyświetlaj tylko, jeśli print_serial == True
                print(f"Serial: {line}")

# Asynchroniczna funkcja główna
async def async_main():
    global ser, print_serial
    # Wybór portu szeregowego
    ports = [port.device for port in serial.tools.list_ports.comports()]
    print("Dostępne porty:", ports)
    port = input("Wybierz port (np. COM0): ")
    if port in ports:
        ser = serial.Serial(port, 115200)
        threading.Thread(target=read_serial, daemon=True).start()
        print(f"Połączono z portem szeregowym: {port}")
    else:
        print("Nieprawidłowy port, pomijam połączenie szeregowe.")

    # Połączenie BLE
    await connect_ble()

    # Pętla do wysyłania komend
    print("Wpisz komendę (np. '1', '2', '3', '4', '5', 'wl', 'wyl') lub 'exit' aby zakończyć:")
    while True:
        cmd = input("Komenda: ")
        if cmd.lower() == 'exit':
            break
        elif cmd.lower() == 'wl':
            print_serial = True
            print("Włączono wyświetlanie danych z portu szeregowego")
            continue
        elif cmd.lower() == 'wyl':
            print_serial = False
            print("Wyłączono wyświetlanie danych z portu szeregowego")
            continue
        if cmd:
            await send_command(cmd)

    # Rozłączenie przed zakończeniem
    if client and client.is_connected:
        await client.disconnect()
        print("Rozłączono z urządzeniem BLE")

# Funkcja główna
def main():
    # Uruchomienie asynchronicznej pętli
    asyncio.run(async_main())

if __name__ == "__main__":
    main()