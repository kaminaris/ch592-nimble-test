#include <Arduino.h>
#include <Uart.h>
#include <stdarg.h>
#include <NimBLEDevice.h>
#include "CH59x_uart.h"
#include <utils/debug_utils.h>

// Uart Serials(&R32_UART0_CTRL);
volatile uint32_t loopCounter = 0;
auto ledState                 = LOW;

void UART0_SendString(uint8_t* buf, uint16_t l) {
	uint16_t len = l;

	while (len) {
		if (R8_UART0_TFC != UART_FIFO_SIZE) {
			R8_UART0_THR = *buf++;
			len--;
		}
	}
}

void UART0_DefInit(void) {
	UART0_BaudRateCfg(115200);
	R8_UART0_FCR = (2 << 6) | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN; // FIFO�򿪣�������4�ֽ�
	R8_UART0_LCR = RB_LCR_WORD_SZ;
	R8_UART0_IER = RB_IER_TXD_EN;
	R8_UART0_DIV = 1;
}

uint32_t GetSysClock(void) {
	uint16_t rev;

	rev = R32_CLK_SYS_CFG & 0xff;
	if ((rev & 0x40) == (0 << 6)) {
		// 32M���з�Ƶ
		return (32000000 / (rev & 0x1f));
	}
	else if ((rev & RB_CLK_SYS_MOD) == (1 << 6)) {
		// PLL���з�Ƶ
		return (480000000 / (rev & 0x1f));
	}
	else {
		// 32K����Ƶ
		return (32000);
	}
}

void UART0_BaudRateCfg(uint32_t baudrate) {
	uint32_t x;

	x            = 10 * GetSysClock() / 8 / baudrate;
	x            = (x + 5) / 10;
	R16_UART0_DL = (uint16_t)x;
}

char buf[64];

static void mini_itoa(int val, char* buf, int base) {
	char* p = buf;
	if (val < 0 && base == 10) {
		*p++ = '-';
		val  = -val;
	}
	char tmp[16],* tp = tmp;
	do { *tp++ = "0123456789abcdef"[val % base]; } while (val /= base);
	while (tp > tmp) *p++ = *--tp;
	*p = 0;
}

int mini_vsnprintf(char* buf, size_t size, const char* fmt, va_list args) {
	char* p   = buf;
	char* end = buf + size - 1;

	while (*fmt && p < end) {
		if (*fmt == '%') {
			fmt++;
			if (*fmt == 'd') {
				char tmp[16];
				mini_itoa(va_arg(args, int), tmp, 10);
				char* s = tmp;
				while (*s && p < end) *p++ = *s++;
			}
			else if (*fmt == 'x') {
				char tmp[16];
				mini_itoa(va_arg(args, int), tmp, 16);
				char* s = tmp;
				while (*s && p < end) *p++ = *s++;
			}
			else if (*fmt == 's') {
				char* s = va_arg(args, char*);
				while (*s && p < end) *p++ = *s++;
			}
			else if (*fmt == 'c') {
				*p++ = va_arg(args, int);
			}
			fmt++;
		}
		else {
			*p++ = *fmt++;
		}
	}
	*p = 0;
	return p - buf;
}

extern "C" {
int __wrap_printf(const char* f, ...) { return 0; }
int __wrap_vprintf(const char* f, va_list a) { return 0; }
int __wrap_sprintf(char* s, const char* f, ...) { return 0; }
int __wrap_vsprintf(char* s, const char* f, va_list a) { return 0; }
int __wrap_asprintf(char** s, const char* f, ...) { return 0; }
int __wrap_vasprintf(char** s, const char* f, va_list a) { return 0; }

int __wrap_snprintf(char* s, size_t n, const char* f, ...) {
	va_list args;
	va_start(args, f);
	int ret = mini_vsnprintf(s, n, f, args);
	va_end(args);
	return ret;
}

int __wrap_vsnprintf(char* s, size_t n, const char* f, va_list a) {
	return mini_vsnprintf(s, n, f, a);
}
}

void Serialprintf(const char* format, ...) {
	va_list args;
	va_start(args, format);
	mini_vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);
	UART0_SendString((uint8_t*)buf, strlen(buf));
}


class ServerCallbacks: public NimBLEServerCallbacks {
	void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
		Serialprintf("Client address: %s\n", connInfo.getAddress().toString().c_str());

		/**
		 *  We can use the connection handle here to ask for different connection parameters.
		 *  Args: connection handle, min connection interval, max connection interval
		 *  latency, supervision timeout.
		 *  Units; Min/Max Intervals: 1.25 millisecond increments.
		 *  Latency: number of intervals allowed to skip.
		 *  Timeout: 10 millisecond increments.
		 */
		pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 180);
	}

	void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
		Serialprintf("Client disconnected - start advertising\n");
		NimBLEDevice::startAdvertising();
	}

	void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) override {
		Serialprintf("MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());
	}

	/********************* Security handled here *********************/
	uint32_t onPassKeyDisplay() override {
		Serialprintf("Server Passkey Display\n");
		/**
		 * This should return a random 6 digit number for security
		 *  or make your own static passkey as done here.
		 */
		return 123456;
	}

	void onConfirmPassKey(NimBLEConnInfo& connInfo, uint32_t pass_key) override {
		Serialprintf("The passkey YES/NO number: %" PRIu32 "\n", pass_key);
		/** Inject false if passkeys don't match. */
		NimBLEDevice::injectConfirmPasskey(connInfo, true);
	}

	void onAuthenticationComplete(NimBLEConnInfo& connInfo) override {
		/** Check that encryption was successful, if not we disconnect the client */
		if (!connInfo.isEncrypted()) {
			NimBLEDevice::getServer()->disconnect(connInfo.getConnHandle());
			Serialprintf("Encrypt connection failed - disconnecting client\n");
			return;
		}

		Serialprintf("Secured connection to: %s\n", connInfo.getAddress().toString().c_str());
	}
} serverCallbacks;

/** Handler class for characteristic actions */
class CharacteristicCallbacks: public NimBLECharacteristicCallbacks {
	void onRead(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
		Serialprintf(
			"%s : onRead(), value: %s\n",
			pCharacteristic->getUUID().toString().c_str(),
			pCharacteristic->getValue().c_str()
		);
	}

	void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
		Serialprintf(
			"%s : onWrite(), value: %s\n",
			pCharacteristic->getUUID().toString().c_str(),
			pCharacteristic->getValue().c_str()
		);
	}

	/**
	 *  The value returned in code is the NimBLE host return code.
	 */
	void onStatus(NimBLECharacteristic* pCharacteristic, int code) override {
		Serialprintf("Notification/Indication return code: %d, %s\n", code, NimBLEUtils::returnCodeToString(code));
	}

	/** Peer subscribed to notifications/indications */
	void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) override {
		// std::string str = "Client ID: ";
		// str             += connInfo.getConnHandle();
		// str             += " Address: ";
		// str             += connInfo.getAddress().toString();
		// if (subValue == 0) {
		// 	str += " Unsubscribed to ";
		// }
		// else if (subValue == 1) {
		// 	str += " Subscribed to notifications for ";
		// }
		// else if (subValue == 2) {
		// 	str += " Subscribed to indications for ";
		// }
		// else if (subValue == 3) {
		// 	str += " Subscribed to notifications and indications for ";
		// }
		// str += std::string(pCharacteristic->getUUID());

		// Serialprintf("%s\n", str.c_str());
	}
} chrCallbacks;

/** Handler class for descriptor actions */
class DescriptorCallbacks: public NimBLEDescriptorCallbacks {
	void onWrite(NimBLEDescriptor* pDescriptor, NimBLEConnInfo& connInfo) override {
		std::string dscVal = pDescriptor->getValue();
		Serialprintf("Descriptor written value: %s\n", dscVal.c_str());
	}

	void onRead(NimBLEDescriptor* pDescriptor, NimBLEConnInfo& connInfo) override {
		Serialprintf("%s Descriptor read\n", pDescriptor->getUUID().toString().c_str());
	}
} dscCallbacks;

static NimBLEServer* pServer;
void setup() {
	// UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);
	// PrintHex(watermark);
	PrintHex(0xaBaaaaa0);
	pinMode(PA8, OUTPUT);
	PrintHex(0xaBaaaaa1);
	digitalWrite(PA8, HIGH); // Force known state first

	PrintHex(0xaBaaaaa2);
	// delay(1000);
	PrintHex(0xaBaaaaa3);
	digitalWrite(PA8, LOW);
	PrintHex(0xaBaaaaa4);
	pinMode(PB4, INPUT); // RX0 (PB4)
	pinMode(PB7, OUTPUT); // TX0 (PB7)
	UART0_DefInit();

	PrintHex(0xaBaaaaa5);
	// watermark = uxTaskGetStackHighWaterMark(NULL);
	// PrintHex(watermark);
	NimBLEDevice::init("NimBLECH59x");
	pServer = NimBLEDevice::createServer();
	pServer->setCallbacks(&serverCallbacks);

	// NimBLEService* pDeadService               = pServer->createService("DEAD");
	// NimBLECharacteristic* pBeefCharacteristic = pDeadService->createCharacteristic(
	// 	"BEEF",
	// 	READ | WRITE
	// );
	//
	// pBeefCharacteristic->setValue("Burger");
	// pBeefCharacteristic->setCallbacks(&chrCallbacks);
	//
	// NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
	// pAdvertising->setName("NimBLEServ");
	// pAdvertising->addServiceUUID(pDeadService->getUUID());
	// /**
	//  *  If your device is battery powered you may consider setting scan response
	//  *  to false as it will extend battery life at the expense of less data sent.
	//  */
	// // pAdvertising->enableScanResponse(true);
	// pAdvertising->start();

}

void loop() {
	loopCounter++;
	// Serials.println("Looping...");
	ledState = (ledState == LOW) ? HIGH : LOW;
	digitalWrite(PA8, ledState);
	Serialprintf("Hello %d\n", loopCounter);
	delay(500);
}

extern "C" {
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
	volatile const char* task = pcTaskName;
	while (1) {
	} // Halt here - inspect stackOverflow variable
}

void vApplicationMallocFailedHook(void) {
	volatile int mallocFailed = 1;
	while (1) {
	} // Halt here
}
//
// void vApplicationIdleHook(void) {
// 	static volatile uint32_t idleCounter = 0;
// 	idleCounter++; // Watch this increment
// }

void abort(void) {
	Serialprintf("ABORT called!");
	while (1); // Stop here in debugger
}
}
