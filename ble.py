from bleak import BleakScanner
import asyncio

async def detect_ble_advertisements():
	def callback(device, advertising_data):
		# Raw advertisement data
		print(f"Device: {device.address}")
		print(f"RSSI: {advertising_data.rssi}")
		print(f"Local name: {advertising_data.local_name}")
		print(f"Manufacturer data: {advertising_data.manufacturer_data}")
		print(f"Service data: {advertising_data.service_data}")
		print("---")

	scanner = BleakScanner(callback)
	await scanner.start()
	await asyncio.sleep(5.0)
	await scanner.stop()

asyncio.run(detect_ble_advertisements())