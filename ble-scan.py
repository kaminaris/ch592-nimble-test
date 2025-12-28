# python
import asyncio
from bleak import BleakScanner

MATCH_NAMES = ("nimbleserv", "nimblech59x", "nimble")
MATCH_UUID_FRAGMENT = "dead"  # matches 16-bit "DEAD" or full UUID containing "dead"

async def scan_and_list(duration: float = 5.0):
	devices = await BleakScanner.discover(timeout=duration)
	if not devices:
		print("No BLE devices found.")
		return

	print(f"Found {len(devices)} device(s):\n")
	for d in devices:
		addr = getattr(d, "address", "")
		name = (getattr(d, "name", "") or "")  # might be None
		rssi = getattr(d, "rssi", None)

		# Different bleak versions expose advertisement info in .metadata
		uuids = []
		mfg = {}
		if hasattr(d, "metadata") and isinstance(d.metadata, dict):
			uuids = d.metadata.get("uuids") or []
			mfg = d.metadata.get("manufacturer_data") or {}
		elif hasattr(d, "details") and isinstance(d.details, dict):
			uuids = d.details.get("uuids") or []

		is_guess = (
				any(m in name.lower() for m in MATCH_NAMES)
				or any(MATCH_UUID_FRAGMENT in (u or "").lower() for u in uuids)
		)

		print(f"Address: {addr}")
		print(f"  Name: {name!s}")
		print(f"  RSSI: {rssi}")
		print(f"  Service UUIDs: {uuids}")
		print(f"  Manufacturer Data: {mfg}")
		if is_guess:
			print("  -> Possible match (name or service UUID resembles NimBLE/DEAD)")
		print()

if __name__ == "__main__":
	asyncio.run(scan_and_list(duration=6.0))
