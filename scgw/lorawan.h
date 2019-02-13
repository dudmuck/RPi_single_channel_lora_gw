#include <stdarg.h>
#include "i2c_lora.h"

#define RECEIVE_DELAY_ms        100

#define DEVADDR_NONE        0xffffffff

#define LORA_EUI_LENGTH                 8
#define LORA_CYPHERKEYBYTES             16


typedef struct {
    uint8_t show_mac      : 1; // 0
    uint8_t show_app      : 1; // 1
    uint8_t unused        : 6; // 2,3,4,5,6,7
} flags_t;

typedef enum
{
    MAC,
    APP,
    BOTH
} layer_e;

typedef struct ota_mote_t ota_mote_t;

struct _mote_list {
    ota_mote_t* motePtr;    /**< this mote */
    struct _mote_list* next;    /**< next mote */
};

#define MAC_CMD_QUEUE_SIZE      6
#define MAC_CMD_SIZE            8
struct ota_mote_t {
    uint8_t dev_eui[LORA_EUI_LENGTH];
    uint8_t app_key[LORA_CYPHERKEYBYTES];

    uint8_t app_session_key[LORA_CYPHERKEYBYTES];
    uint8_t network_session_key[LORA_CYPHERKEYBYTES];
    uint32_t dev_addr;

    uint16_t tx_slot_offset;

    uint8_t macCmd_queue[MAC_CMD_QUEUE_SIZE][MAC_CMD_SIZE];
    uint8_t macCmd_queue_in_idx, macCmd_queue_out_idx;

    uint8_t user_downlink_length;
    uint8_t user_downlink_port;
    uint8_t user_downlink[244];
    uint16_t FCntDown;
};

class LoRaWan {
    private:
        static void SendJoinComplete(uint16_t deviceNonce, uint8_t firstReceiveWindowDataRateoffset, ota_mote_t* mote);
        static void parse_mac_command(ota_mote_t* mote, const uint8_t* rx_cmd_buf, uint8_t rx_cmd_buf_len);
        static void parse_uplink(ota_mote_t* mote, uint8_t rxSize, const uint8_t*);
        static void parse_join_req(ota_mote_t* mote, uint8_t rxSize, const uint8_t*);
        static void classA_downlink(ota_mote_t* mote, uint8_t*, uint8_t);

    public:
        /* \brief call from radio RxDone event */
        static int parse_receive(uint8_t rxSize, float rssi, float snr, const uint8_t*);

        /* \brief initialize lora mac layer */
        static void init(uint64_t);
        static void print_octets_rev(char const* label, uint8_t const* buf, uint8_t buf_len);

        static volatile uint16_t rx_slot;
        static volatile uint32_t beaconDur; /**< stores the duration of beacon: measured when beacon is configured */
        static uint32_t dev_addr_filter;
        static unsigned SLOT_STEPPING; /**< time given to each end device (for uplink with downlink time), in 30ms steps */
        static unsigned PERIODICITY_SLOTS_; /**< number of end devices on network */
        static void filtered_printf(uint32_t dev_addr, layer_e, const char* format, ...);
        static void filtered_vprintf(uint32_t dev_addr, layer_e, const char* format, va_list);

        static volatile flags_t flags;
        static uint64_t appEui64;
		static struct _mote_list* mote_list;
};


void send_downlink(void);
void decrypted_uplink(const uint8_t* devEui, uint32_t dev_addr, const uint8_t* buf, uint8_t buflen, uint8_t port);
