// Dual MRF24J40MA broadcast example
#include <SPI.h>
#include <MRF24J40.h>

const int PIN_MRF_CS_1 = 9; // SPI Chip-Select pin for the first MRF24J40MA
const int PIN_MRF_CS_2 = 10; // SPI Chip-Select pin for the second MRF24J40MA

Mrf24j40 mrf1(PIN_MRF_CS_1); // Will receive broadcasts
Mrf24j40 mrf2(PIN_MRF_CS_2); // Will send broadcasts

// Network Configuration
const uint8_t CHANNEL = 12; // Send and receive packets on this channel
const uint16_t PAN_ID = 0xCAFE; // Network ID
const uint16_t ADDR1 = 0x6001; // Address for first MRF24J40MA
const uint16_t ADDR2 = 0x6002; // Address for second MRF24J40MA

// IEEE 802.15.4 Broadcast Address
const uint16_t BROADCAST_ADDR = 0xFFFF;

// IEEE 802.15.4 MAC Frame - Header size
const int DATA_HEADER_SIZE = 9;
const int ACK_HEADER_SIZE = 3;
const int FCS_SIZE = 2;

// IEEE 802.15.4 MAC Frame - Frame Control 0
#define FC0_INTRA_PAN         (1<<6) // Destination within the same PAN
#define FC0_ACK_REQUEST       (1<<5) // Receiver should respond with ACK
#define FC0_FRAME_PENDING     (1<<4) // ???
#define FC0_SECURITY_ENABLED  (1<<3) // Encrypt frame
#define FC0_FRAME_TYPE_BEACON (0x00)
#define FC0_FRAME_TYPE_DATA   (0x01)
#define FC0_FRAME_TYPE_ACK    (0x02)
#define FC0_FRAME_TYPE_MASK   (0x03)

// IEEE 802.15.4 MAC Frame - Frame Control 1
#define FC1_SOURCE_NONE       (0x00<<6)
#define FC1_SOURCE_SHORT      (0x02<<6)
#define FC1_SOURCE_LONG       (0x03<<6)
#define FC1_SOURCE_MASK       (0x03<<6)
#define FC1_DEST_NONE         (0x00<<2)
#define FC1_DEST_SHORT        (0x02<<2)
#define FC1_DEST_LONG         (0x03<<2)
#define FC1_DEST_MASK         (0x03<<2)

void setup()
{
	while (!Serial) {}
	Serial.begin(9600);
	Serial.println("mrf24j40 example");

	mrf1.begin();
	mrf2.begin();

	mrf1.set_channel(CHANNEL); // Unique per network
	mrf1.set_pan(PAN_ID); // Unique per network
 	mrf1.set_short_addr(ADDR1); // Unique per device
 	mrf1.set_promiscuous(false); // Can receive when set to false

 	mrf2.set_channel(CHANNEL); // Unique per network
	mrf2.set_pan(PAN_ID); // Unique per network
 	mrf2.set_short_addr(ADDR2); // Unique per device
 	mrf2.set_promiscuous(false); // Can receive when set to false

	Serial.println("Ready");
}

void loop()
{
	// Poll modules and react to events (no interrupts are used here, and
	// therefore, it is almost impossible to respond to ACKs quick enough)
	runTasks(mrf1, "MRF #1: receiver");
	runTasks(mrf2, "MRF #2: transmitter");

	// Send a broadcast packet periodically
	static unsigned long lastSent = 0;
	if (millis() - lastSent > 5000) {
		lastSent = millis();

		// Broadcast packets are simply regular packets sent to a special
		// broadcast address
		uint16_t destination = BROADCAST_ADDR;
		const char *payload = "hello world";
		sendData(mrf2, destination, (const uint8_t *)payload, strlen(payload));

		Serial.println("Packet put on FIFO");
	}
}

void sendData(Mrf24j40 &mrf, uint16_t address, const uint8_t *payload,
	uint16_t num_bytes)
{
	static uint8_t sequence_number = 0;

	uint8_t frame[DATA_HEADER_SIZE + num_bytes];

	uint16_t pan = mrf.pan();
	uint16_t source = mrf.short_addr();

	// Frame control
	//   [0 | intra pan | ack | frame pending | security | frame type<2:0>]
	//   [source. addressing mode<1:0> | 00 | dest. addressing mode<1:0> | 0]
	frame[0] = FC0_FRAME_TYPE_DATA | FC0_INTRA_PAN;
	frame[1] = FC1_SOURCE_SHORT | FC1_DEST_SHORT;

	// Sequence number
	frame[2] = sequence_number++;

	// Destination pan
	frame[3] = pan & 0xFF;
	frame[4] = pan >> 8;

	// Destination address
	frame[5] = address & 0xFF;
	frame[6] = address >> 8;

	// Source address
	frame[7] = source & 0xFF;
	frame[8] = source >> 8;

	// Append payload
	memcpy(frame + DATA_HEADER_SIZE, payload, num_bytes);

	// Add to transmit FIFO
	mrf.txpkt(frame, DATA_HEADER_SIZE, 0, num_bytes);
}

void sendAck(Mrf24j40 &mrf, uint16_t address, uint8_t sequence_number)
{
	uint8_t frame[ACK_HEADER_SIZE];

	// Frame control
	//   [0 | intra pan | ack | frame pending | security | frame type<2:0>]
	//   [source. addressing mode<1:0> | 00 | dest. addressing mode<1:0> | 0]
	frame[0] = FC0_FRAME_TYPE_ACK | FC0_INTRA_PAN;
	frame[1] = FC1_SOURCE_NONE | FC1_DEST_NONE;

	// Sequence number
	frame[2] = sequence_number;

	// Add to transmit FIFO
	mrf.txpkt(frame, ACK_HEADER_SIZE, 0, 0);
}

void runTasks(Mrf24j40 &mrf, const char *name)
{
	// Use the interrupt pin on MRF24J40MA to react faster, if necessary
	uint16_t tasks = mrf.int_tasks();

	if (tasks) {
		Serial.println(name);
	}

	if (tasks & MRF24J40_INT_RX) { // A packet was received
		const int NUM_BYTES = 128;
		uint8_t buffer[NUM_BYTES];

		int read = mrf.rxpkt_intcb(buffer, NUM_BYTES, NULL, NULL);
		Serial.print("Packet received: ");
		Serial.print(read);
		Serial.println(" bytes");

		// Expect frames produced by sendData()
		if (read >= 3) {
			uint8_t frame_control0 = buffer[0];
			uint8_t frame_control1 = buffer[1];
			uint8_t sequence_number = buffer[2];

			uint8_t frame_type = frame_control0 & FC0_FRAME_TYPE_MASK;
			bool is_data_frame = frame_type == FC0_FRAME_TYPE_DATA;
			bool is_short_addr = frame_control1 == FC1_SOURCE_SHORT | FC1_DEST_SHORT;

			Serial.print("  type=");
			Serial.print((int)frame_type, HEX);
			Serial.print(", sequence=");
			Serial.print((int)sequence_number);

			if (is_data_frame && is_short_addr) {
				uint16_t pan = buffer[3] | (buffer[4] << 8);
				uint16_t dest = buffer[5] | (buffer[6] << 8);
				uint16_t source = buffer[7] | (buffer[8] << 8);

				uint8_t *payload = buffer + DATA_HEADER_SIZE;
				uint16_t num_bytes = read - DATA_HEADER_SIZE - FCS_SIZE;

				if (IEEE_802_15_4_WANTS_ACK(frame_control0)) {
					// Without interrupts there is little to no chance this ACK
					// will reach the source fast enough
					sendAck(mrf, source, sequence_number);
				}

				Serial.print(", pan=");
				Serial.print(pan, HEX);
				Serial.print(", source=");
				Serial.print(source, HEX);

				Serial.print("\n  data=");
				for (int i = 0; i < num_bytes; i++) {
					Serial.print(payload[i], HEX);
					Serial.print(' ');
				}
				Serial.print("\n  text=\"");
				for (int i = 0; i < num_bytes; i++) {
					char c = payload[i];
					if (isPrintable(c)) {
						Serial.print(c);
					}
				}
				Serial.println('"');
			}
		}
	}

	if (tasks & MRF24J40_INT_TX) { // A packet was sent
		int16_t status = mrf.txpkt_intcb();
		Serial.print("Packet sent: ");
		if (status == 0) {
			Serial.println("OK");
		}
		else if (status == EIO) {
			Serial.println("FAILED");
		}
		else if (status == EBUSY) {
			Serial.println("BUSY");
		}
	}

	if (tasks & MRF24J40_INT_SEC) {
		mrf.sec_intcb(false);
	}
}

bool isPrintable(char c) {
	return c >= 32 && c < 127;
}
