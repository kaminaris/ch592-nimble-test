from scapy.all import *
from scapy.layers.bluetooth import *

def packet_handler(pkt):
	if BTLE in pkt:
		# Access Link Layer header
		if BTLE_ADV in pkt:
			adv = pkt[BTLE_ADV]
			print(f"Advertiser Address: {adv.AdvA}")

			# Raw bytes
			raw_data = bytes(pkt)
			print(f"Raw frame: {raw_data.hex()}")

			# Parse specific fields
			if hasattr(pkt, 'data'):
				print(f"Payload: {pkt.data}")

# Sniff on Bluetooth interface (requires proper setup)
sniff(iface="hci0", prn=packet_handler, store=0)