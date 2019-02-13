#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <json-c/json.h>
#include "lorawan.h"

#include "scgw.h"

#define BEACON_PRELOAD_us               500000
unsigned n_rssi_samples;

float starting_bg_rssi;
volatile uint32_t usingChHz;


unsigned firstChannelHz;
unsigned channelStepHz;
uint8_t numChannels;

int channel_scan()
{
    int ret = -1, min_ch, ch;
    uint32_t hz = firstChannelHz;
    float min = FLT_MIN;
    int result;
    float* rssi = (float*)malloc(sizeof(float) * numChannels);

    printf("channel_scan: ");
    result = Radio::Standby( );
    if (result < 0) {
        printf("%d = Standby()\r\n", result);
        goto scanDone;
    }
    usleep(20000);
    
    for (ch = 0; ch < numChannels; ch++) {
        printf("setting to %uhz\r\n", hz);
        Radio::SetChannel(hz);
        Radio::Rx(0);
        rssi[ch] = Radio::GetAmbient(n_rssi_samples);
        if (rssi[ch] == FLT_MIN) {
            printf("getAmbient fail\n");
            goto scanDone;
        }
        Radio::Standby( );
        printf("ch%u: %.1f\r\n", ch, rssi[ch]);
        hz += channelStepHz ;
        usleep(20000);
        Radio::SetChannel(hz);
    }

    min_ch = 0;
    for (ch = 0; ch < numChannels; ch++) {
        printf("%u) %.1f ", ch, rssi[ch]);
        if (rssi[ch] < min) {
            printf("*");
            min = rssi[ch];
            min_ch = ch;
        }
        printf("\r\n");
    }
    hz = firstChannelHz + (min_ch * channelStepHz);
    printf("using ch%u, %uhz\r\n", min_ch, hz);
    Radio::SetChannel(hz);
    usingChHz = hz;
    
    starting_bg_rssi = rssi[min_ch];
    ret = 0;

scanDone:
    free(rssi);
    return ret;
}

void measure_ambient()
{
    float bg_rssi;
    float diff;
    static unsigned cnt = 0;
    
    bg_rssi = Radio::GetAmbient(n_rssi_samples);
    diff = bg_rssi - starting_bg_rssi;
    printf("bg_rssi:%.1fdBm vs %1.fdBm, diff:%.1f, %d\r\n", bg_rssi, starting_bg_rssi, diff, cnt);
    if (diff > 10) {
        if (++cnt > 3) {
            /* find better channel */
            channel_scan();
            Radio::Rx(0);
            cnt = 0;
        }
    } else
        cnt = 0;
}

static void OnRadioRxDone(uint16_t slot, uint8_t size, float rssi, float snr, const uint8_t* rx_buf)
{
    LoRaWan::rx_slot = slot;

    LoRaWan::parse_receive(size, rssi, snr, rx_buf);
}

void ambient_callback(float bg_rssi)
{
    static unsigned cnt = 0;
    float diff = bg_rssi - starting_bg_rssi;
    printf("bg_rssi:%.1fdBm vs %1.fdBm, diff:%.1f, %d\r\n", bg_rssi, starting_bg_rssi, diff, cnt);
    if (diff > 10) {
        if (++cnt > 3) {
            /* find better channel */
            channel_scan();
            Radio::Rx(0);
            cnt = 0;
        }
    } else
        cnt = 0;
}

static int init_radio(int txDBM, uint8_t sf, unsigned bw_khz)
{
    cfg_t cfg;
    int ret;
    memset(&cfg, 0, sizeof(cfg_t));

    cfg.fields.flags.rxOnTxDone = 1;
    cfg.fields.n_rssi_samples = n_rssi_samples;
    cfg.fields.downlinkOffset = RECEIVE_DELAY_ms;
    ret = Radio::Config(&cfg);
    if (ret < 0) {
        printf("%d = Config()\r\n", ret);
        return ret;
    }

    Radio::OnRadioRxDone = OnRadioRxDone;
    Radio::background_rssi = ambient_callback;

    Radio::Standby( );

    Radio::set_tx_dbm(txDBM);

    Radio::LoRaModemConfig(bw_khz, sf, 1);
    printf("using sf%u\r\n", sf);
    Radio::SetPublicNetwork(true);

    ret = channel_scan();

    Radio::SetRxMaxPayloadLength(255);

    return ret;
}

void scgw_stop()
{
    Radio::i2c_lora_close();
}

static uplink_payload_t uplink_callback;
unsigned serviceRate_us;

static int load_json_config(const char* conf_file)
{
    unsigned irqPin, preambleSymbs, i2c_slave_address, beaconInterval, beaconSize;
    unsigned bwKHz;
    unsigned spreadingFactor;
    int tx_dbm, len, ret = -1;
    uint64_t appeui;
    json_object *top, *obj;
    enum json_tokener_error jerr;
    struct json_tokener *tok = json_tokener_new();
    FILE *file;
    char str[64];

    if (conf_file == NULL)
        conf_file = "../gw_conf.json";

    printf("opening json config \"%s\"\r\n", conf_file);

    file = fopen(conf_file, "r");
    if (file == NULL) {
        perror(conf_file);
        return -1;
    }

    do {
        char line[96];
        if (fgets(line, sizeof(line), file) == NULL) {
            fprintf(stderr, "NULL == fgets()\n");
            goto pEnd;
        }
        len = strlen(line);
        top = json_tokener_parse_ex(tok, line, len);
        //printf("jobj:%p, len%u\n", top, len);
    } while ((jerr = json_tokener_get_error(tok)) == json_tokener_continue);

    if (jerr != json_tokener_success) {
        printf("parse_server_config() json error: %s\n", json_tokener_error_desc(jerr));
        goto pEnd;
    }

    if (json_object_object_get_ex(top, "irqPin", &obj)) {
        irqPin = json_object_get_int(obj);
    } else {
        printf("need irqPin\n");
        goto pEnd;
    }

    if (json_object_object_get_ex(top, "i2c_slave_address", &obj)) {
        sscanf(json_object_get_string(obj), "%x", &i2c_slave_address);
    } else {
        printf("need i2c_slave_address\n");
        goto pEnd;
    }

    if (json_object_object_get_ex(top, "preambleSymbs", &obj)) {
        preambleSymbs = json_object_get_int(obj);
    } else {
        printf("need preambleSymbs\n");
        goto pEnd;
    }

    if (json_object_object_get_ex(top, "serviceRate_us", &obj)) {
        serviceRate_us = json_object_get_int(obj);
    } else {
        printf("need serviceRate_us\n");
        goto pEnd;
    }

    if (json_object_object_get_ex(top, "beaconSize", &obj)) {
        beaconSize = json_object_get_int(obj);
    } else {
        printf("need beaconSize\n");
        goto pEnd;
    }

    if (json_object_object_get_ex(top, "beaconInterval", &obj)) {
        beaconInterval = json_object_get_int(obj);
    } else {
        printf("need beaconInterval\n");
        goto pEnd;
    }

    if (json_object_object_get_ex(top, "appEui", &obj)) {
        sscanf(json_object_get_string(obj), "%" PRIx64, &appeui);
        printf("appEui: %016" PRIx64 "\n", appeui);
    } else {
        printf("need appEui\n");
        goto pEnd;
    }

    if (json_object_object_get_ex(top, "tx_dbm", &obj)) {
        tx_dbm = json_object_get_int(obj);
    } else {
        printf("need tx_dbm\n");
        goto pEnd;
    }

    if (json_object_object_get_ex(top, "rssiSamples", &obj)) {
        n_rssi_samples = json_object_get_int(obj);
    } else {
        printf("need rssiSamples\n");
        goto pEnd;
    }

    if (json_object_object_get_ex(top, "motes", &obj)) {
        unsigned i, n = json_object_array_length(obj); 
        for (i = 0; i < n; i++) {
            const char* devEuistr = NULL;
            const char* rootKey = NULL;
            json_object *o, *ajo = json_object_array_get_idx(obj, i);
            if (json_object_object_get_ex(ajo, "devEui", &o)) {
                devEuistr = json_object_get_string(o);
            }
            if (json_object_object_get_ex(ajo, "key", &o)) {
                rootKey = json_object_get_string(o);
            }
            printf("adding mote %s %s\r\n", devEuistr, rootKey);
            scgw_add_mote(devEuistr, rootKey);
        }
    } // ..if motes object exists

    if (json_object_object_get_ex(top, "band", &obj)) {
        json_object *band_obj;
        const char* band = json_object_get_string(obj);
        if (json_object_object_get_ex(top, band, &band_obj)) {
            if (json_object_object_get_ex(band_obj, "spreadingFactor", &obj)) {
                spreadingFactor = json_object_get_int(obj);
            } else {
                printf("need spreadingFactor\n");
                goto pEnd;
            }
            if (json_object_object_get_ex(band_obj, "bwKHz", &obj)) {
                bwKHz = json_object_get_int(obj);
            } else {
                printf("need bwKHz\n");
                goto pEnd;
            }
            if (json_object_object_get_ex(band_obj, "firstChannelHz", &obj)) {
                firstChannelHz = json_object_get_int(obj);
            }
            if (json_object_object_get_ex(band_obj, "channelStepHz", &obj)) {
                channelStepHz = json_object_get_int(obj);
            }
            if (json_object_object_get_ex(band_obj, "numChannels", &obj)) {
                numChannels = json_object_get_int(obj);
            }
        } else {
            printf("need %s\n", band);
            goto pEnd;
        }
    } else {
        printf("need band\n");
        goto pEnd;
    }

    sprintf(str, "timing%uKHz", bwKHz);
    if (json_object_object_get_ex(top, str, &obj)) {
        json_object *tobj;
        sprintf(str, "sf%u", spreadingFactor);
        if (json_object_object_get_ex(obj, str, &tobj)) {
            if (json_object_object_get_ex(tobj, "SLOT_STEPPING", &obj)) {
                LoRaWan::SLOT_STEPPING = json_object_get_int(obj);
            } else {
                printf("need SLOT_STEPPING in %s\n", str);
                goto pEnd;
            }
            if (json_object_object_get_ex(tobj, "PERIODICITY_SLOTS", &obj)) {
                LoRaWan::PERIODICITY_SLOTS_ = json_object_get_int(obj);
            } else {
                printf("need PERIODICITY_SLOTS in %s\n", str);
                goto pEnd;
            }
        } else {
            printf("need %s\n", str);
            goto pEnd;
        }

    } else {
        printf("need %s\n", str);
        goto pEnd;
    }

    printf("slave address: 0x%02x irqPin:%u\r\n", i2c_slave_address, irqPin);
    /********* config load done, config slave: *********/
    if (Radio::i2c_lora_setup(i2c_slave_address, irqPin) < 0) {
        printf("i2c_lora_setup fail\n");
        return -1;
    }

    if (init_radio(tx_dbm, spreadingFactor, bwKHz) < 0) {
        printf("init_radio() failed\r\n");
        return -1;
    }

    ret = Radio::BeaconCfg(beaconInterval, preambleSymbs, beaconSize);
    if (ret < 0)
        return -1;
    LoRaWan::beaconDur = ret;
    printf("beaconDur:%u\r\n", LoRaWan::beaconDur);

           //    preambleLen, fixLen, crcOn, invIQ
    Radio::LoRaPacketConfig(preambleSymbs, false, true, false);
    Radio::Rx(0);

    LoRaWan::init(appeui);

    ret = 0;

pEnd:
    if (tok)
        json_tokener_free(tok);
    fclose(file);
    return ret;

}

int scgw_start(const char* confFile, uplink_payload_t callback)
{
    if (load_json_config(confFile) < 0) {
        printf("load_json_config() failed\r\n");
        return -1;
    }

    /* default to printing both mac layer and application layer */
    LoRaWan::flags.show_mac = 1;
    LoRaWan::flags.show_app = 1;

    uplink_callback = callback;

    return 0;
}

void scgw_service()
{
    Radio::service();
}

void scgw_add_mote(const char* devEuistr, const char* rootKey)
{
    struct _mote_list* my_mote_list;
    unsigned oct, x;
    ota_mote_t* mote;

    if (LoRaWan::mote_list == NULL) {
        LoRaWan::mote_list = (struct _mote_list*)malloc(sizeof(struct _mote_list));
        my_mote_list = LoRaWan::mote_list;
    } else {
        for (my_mote_list = LoRaWan::mote_list; my_mote_list->next != NULL; my_mote_list = my_mote_list->next)
            ;
        my_mote_list->next = (struct _mote_list*)malloc(sizeof(struct _mote_list));
        my_mote_list = my_mote_list->next;
    }
    my_mote_list->motePtr = (ota_mote_t*)calloc(sizeof(ota_mote_t), 1);
    my_mote_list->next = NULL;

    mote = my_mote_list->motePtr;

    for (x = 0; x < LORA_EUI_LENGTH; x++) {
        sscanf(devEuistr, "%02x", &oct);
        mote->dev_eui[x] = oct;
        devEuistr += 2;
    }

    for (x = 0; x < LORA_CYPHERKEYBYTES; x++) {
        sscanf(rootKey, "%02x", &oct);
        mote->app_key[x] = oct;
        rootKey += 2;
    }

    mote->dev_addr = DEVADDR_NONE;
}

void scgw_enable_mac_printf(bool en)
{
    LoRaWan::flags.show_app = en;
}

void scgw_enable_app_printf(bool en)
{
    LoRaWan::flags.show_mac = en;
}

static int _send_downlink(ota_mote_t *mote, const uint8_t* payload, uint8_t paylen, uint8_t port)
{
    if (mote == NULL) {
        printf("mote not found\r\n");
        return -1;
    }

    if (paylen > sizeof(mote->user_downlink)) {
        return -1;
    }
    mote->user_downlink_length = paylen;
    mote->user_downlink_port = port;
    printf("putting downlink of %u onto %08x\r\n", paylen, mote->dev_addr);
    memcpy(mote->user_downlink, payload, mote->user_downlink_length);
    return 0;
}

int scgw_send_downlink(const uint8_t* devEui, const uint8_t* payload, uint8_t paylen, uint8_t port)
{
    ota_mote_t* mote = NULL;
    struct _mote_list* my_mote_list;

    for (my_mote_list = LoRaWan::mote_list; my_mote_list != NULL; my_mote_list = my_mote_list->next) {
        ota_mote_t* _mote = my_mote_list->motePtr;
        if (memcmp(_mote->dev_eui, devEui, LORA_EUI_LENGTH) == 0) {
            mote = _mote;
            break;
        }
    }
    return _send_downlink(mote, payload, paylen, port);
}

int scgw_send_downlink(unsigned dev_addr, const uint8_t* payload, uint8_t paylen, uint8_t port)
{
    ota_mote_t* mote = NULL;
    struct _mote_list* my_mote_list;

    for (my_mote_list = LoRaWan::mote_list; my_mote_list != NULL; my_mote_list = my_mote_list->next) {
        ota_mote_t* _mote = my_mote_list->motePtr;
        if (_mote->dev_addr == dev_addr) {
            mote = _mote;
            break;
        }
    }
    return _send_downlink(mote, payload, paylen, port);
}

void scgw_set_dev_addr_filter(uint32_t filt)
{
    LoRaWan::dev_addr_filter = filt;
}

void scgw_set_tx_dbm(int8_t s8)
{
    Radio::set_tx_dbm(s8);
}

void scgw_set_beacon_skip(uint8_t n)
{
    Radio::SetBeaconSkip(n);
}

uint8_t scgw_get_beacon_skip()
{
    return Radio::GetBeaconSkip();
}

void scgw_cmd_list_motes(uint8_t idx)
{
    struct _mote_list* my_mote_list;

    for (my_mote_list = LoRaWan::mote_list; my_mote_list != NULL; my_mote_list = my_mote_list->next) {
        ota_mote_t* _mote = my_mote_list->motePtr;
        if (_mote->dev_addr != DEVADDR_NONE) {
            LoRaWan::print_octets_rev("", _mote->dev_eui, LORA_EUI_LENGTH);
            printf("    %08" PRIx32 "\r\n", _mote->dev_addr);
        }
    }
}

void scgw_print_status()
{
    int cnt, slot;
    float MHz = Radio::GetCF();
    if (MHz == FLT_MIN) {
        printf("GetCF failed\r\n");
        return;
    }
    if (MHz < 100)
        printf(" \e[41m%fMHz\e[0m, ", MHz); // likely invalid center frequency
    else
        printf(" %fMHz, ", MHz);

    printf("rssi:%.1f\r\n", Radio::GetAmbient(2));
    cnt = Radio::GetBeaconSkip();
    if (cnt < 0) {
        printf("GetBeaconSkip failed\r\n");
        return;
    }
    slot = Radio::GetCurrentSlot();
    if (slot < 0) {
        printf("GetCurrentSlot failed\r\n");
        return;
    }
    printf("\r\nbeacon_skip:%u curSlot:%u\r\n", cnt, slot);
}

void app_printf(uint32_t dev_addr, const char* format, ...)
{
    va_list args;
    va_start(args, format);
    LoRaWan::filtered_vprintf(dev_addr, APP, format, args);
}

void decrypted_uplink(const uint8_t* devEui, uint32_t dev_addr, const uint8_t* buf, uint8_t buflen, uint8_t port)
{
    if (uplink_callback)
        uplink_callback(devEui, dev_addr, buf, buflen, port);
}
