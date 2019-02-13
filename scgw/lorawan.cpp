#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "lorawan.h"

#include "gladman_aes.h"
#include "gladman_cmac.h"

//#define ANY_DEVEUI


#define LORA_FRAMEMICBYTES              4
#define LORA_ENCRYPTIONBLOCKBYTES       16
#define LORA_AUTHENTICATIONBLOCKBYTES   16
#define LORA_MAXFRAMELENGTH             235
#define LORA_MACHEADERLENGTH            1
#define LORA_MINDATAHEADERLENGTH        7
#define LORA_PORTLENGTH                 1
#define LORA_MAXDATABYTES    (LORA_MAXFRAMELENGTH - (LORA_MACHEADERLENGTH + LORA_MINDATAHEADERLENGTH + LORA_PORTLENGTH + LORA_FRAMEMICBYTES)) //excluding port
#define LORA_NETWORKADDRESSBITS         25


const uint32_t network_id = 0x24;
uint32_t networkAddress = 0;  // bits 24..0 of DevAddr, for join accept
uint16_t next_available_tx_slot = 0;

volatile uint16_t LoRaWan::rx_slot;
volatile uint32_t LoRaWan::beaconDur;
volatile flags_t LoRaWan::flags;
uint32_t LoRaWan::dev_addr_filter;
struct _mote_list* LoRaWan::mote_list = NULL;
uint64_t LoRaWan::appEui64;
unsigned LoRaWan::SLOT_STEPPING;
unsigned LoRaWan::PERIODICITY_SLOTS_;


typedef enum {
    MTYPE_JOIN_REQ = 0,
    MTYPE_JOIN_ACC,  // 1
    MTYPE_UNCONF_UP, // 2
    MTYPE_UNCONF_DN, // 3
    MTYPE_CONF_UP,   // 4
    MTYPE_CONF_DN,   // 5
    MTYPE_RFU,       // 6
    MTYPE_P,         // 7
} mtype_e;

typedef union {
    struct {
        uint8_t major   : 2;    // 0 1
        uint8_t rfu     : 3;    // 2 3 4
        uint8_t MType   : 3;    // 5 6 7
    } bits;
    uint8_t octet;
} mhdr_t;

typedef union {
    struct {
        uint8_t FOptsLen        : 4;    // 0 1 2 3
        uint8_t FPending        : 1;    // 4 
        uint8_t ACK             : 1;    // 5
        uint8_t ADCACKReq       : 1;    // 6
        uint8_t ADR             : 1;    // 7
    } dlBits;   // downlink  (gwtx)
    struct {
        uint8_t FOptsLen        : 4;    // 0 1 2 3
        uint8_t classB          : 1;    // 4    unused in classA
        uint8_t ACK             : 1;    // 5
        uint8_t ADCACKReq       : 1;    // 6
        uint8_t ADR             : 1;    // 7
    } ulBits;   // uplink   (gwrx)
    uint8_t octet;
} FCtrl_t;

typedef struct {
    uint32_t DevAddr;
    FCtrl_t FCtrl;
    uint16_t FCnt;
} __attribute__((packed)) fhdr_t;


typedef struct {
    mhdr_t mhdr;
    uint8_t AppEUI[LORA_EUI_LENGTH];
    uint8_t DevEUI[LORA_EUI_LENGTH];
    uint16_t DevNonce;
} __attribute__((packed)) join_req_t;

typedef enum eLoRaMacMoteCmd
{
    /*!
     * LinkCheckReq
     */
    MOTE_MAC_LINK_CHECK_REQ          = 0x02,
    /*!
     * LinkADRAns
     */
    //MOTE_MAC_LINK_ADR_ANS            = 0x03,
    /*!
     * DutyCycleAns
     */
    //MOTE_MAC_DUTY_CYCLE_ANS          = 0x04,
    /*!
     * RXParamSetupAns
     */
    MOTE_MAC_RX_PARAM_SETUP_ANS      = 0x05,
    /*!
     * DevStatusAns
     */
    MOTE_MAC_DEV_STATUS_ANS          = 0x06,
    /*!
     * NewChannelAns
     */
    MOTE_MAC_NEW_CHANNEL_ANS         = 0x07,
    /*!
     * RXTimingSetupAns
     */
    MOTE_MAC_RX_TIMING_SETUP_ANS     = 0x08,
    /*!
     * PingSlotInfoReq
     */
    MOTE_MAC_PING_SLOT_INFO_REQ      = 0x10,
    /*!
     * PingSlotFreqAns
     */
    MOTE_MAC_PING_SLOT_FREQ_ANS      = 0x11,
    /*!
     * BeaconTimingReq
     */
    MOTE_MAC_BEACON_TIMING_REQ       = 0x12,
    /*!
     * BeaconFreqAns
     */
    MOTE_MAC_BEACON_FREQ_ANS         = 0x13,
}LoRaMacMoteCmd_t;

typedef enum eLoRaMacSrvCmd
{
    /*!
     * LinkCheckAns
     */
    SRV_MAC_LINK_CHECK_ANS           = 0x02,
    /*!
     * LinkADRReq
     */
    //SRV_MAC_LINK_ADR_REQ             = 0x03,
    /*!
     * DutyCycleReq
     */
    //SRV_MAC_DUTY_CYCLE_REQ           = 0x04,
    /*!
     * RXParamSetupReq
     */
    SRV_MAC_RX_PARAM_SETUP_REQ       = 0x05,
    /*!
     * DevStatusReq
     */
    SRV_MAC_DEV_STATUS_REQ           = 0x06,
    /*!
     * NewChannelReq
     */
    SRV_MAC_NEW_CHANNEL_REQ          = 0x07,
    /*!
     * RXTimingSetupReq
     */
    SRV_MAC_RX_TIMING_SETUP_REQ      = 0x08,
    /*!
     * PingSlotInfoAns
     */
    SRV_MAC_PING_SLOT_INFO_ANS       = 0x10,
    /*!
     * PingSlotChannelReq
     */
    SRV_MAC_PING_SLOT_CHANNEL_REQ    = 0x11,
    /*!
     * BeaconTimingAns
     */
    SRV_MAC_BEACON_TIMING_ANS        = 0x12,
    /*!
     * BeaconFreqReq
     */
    SRV_MAC_BEACON_FREQ_REQ          = 0x13,
}LoRaMacSrvCmd_t;


mtype_e user_dowlink_mtype = MTYPE_UNCONF_DN;

void print_octets(char const* label, uint8_t const* buf, uint8_t buf_len)
{
    int i;
    printf("%s:", label);
    for (i = 0; i < buf_len; i++)
        printf(" %02x", buf[i]);
//    printf("\n");
}

void LoRaWan::print_octets_rev(char const* label, uint8_t const* buf, uint8_t buf_len)
{
    int i;
    printf("%s:", label);
    for (i = buf_len-1; i >= 0; i--)
        printf(" %02x", buf[i]);
//    printf("\n");
}

void LoRaWan::filtered_vprintf(uint32_t dev_addr, layer_e l, const char* format, va_list ap)
{
    if (dev_addr_filter != 0) {
        if (dev_addr_filter != dev_addr)
            return;
    }

    switch (l) {
        case MAC:
            if (!flags.show_mac)
                return;
            break;
        case APP:
            if (!flags.show_app)
                return;
            break;
        case BOTH:
            break;
    }

    vprintf(format, ap);
    va_end(ap);
}

void LoRaWan::filtered_printf(uint32_t dev_addr, layer_e l, const char* format, ...)
{
    va_list args;

    if (dev_addr_filter != 0) {
        if (dev_addr_filter != dev_addr)
            return;
    }

    switch (l) {
        case MAC:
            if (!flags.show_mac)
                return;
            break;
        case APP:
            if (!flags.show_app)
                return;
            break;
        case BOTH:
            break;
    }

    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}

uint8_t* Write4ByteValue(uint8_t output[], uint32_t input)
{
    uint8_t* ptr = output;

    *(ptr++) = (uint8_t)input,
    input >>= 8;
    *(ptr++) = (uint8_t)input,
    input >>= 8;
    *(ptr++) = (uint8_t)input,
    input >>= 8;
    *(ptr++) = (uint8_t)input;

    return ptr;
}


uint8_t* Write3ByteValue(uint8_t output[], uint32_t input)
{
    uint8_t* ptr = output;

    *(ptr++) = (uint8_t)input,
    input >>= 8;
    *(ptr++) = (uint8_t)input,
    input >>= 8;
    *(ptr++) = (uint8_t)input;

    return ptr;
}

uint8_t* Write2ByteValue(uint8_t output[], uint32_t input)
{
    uint8_t* ptr = output;

    *(ptr++) = (uint8_t)input,
    input >>= 8;
    *(ptr++) = (uint8_t)input;

    return ptr;
}

uint8_t* Write1ByteValue(uint8_t output[], uint32_t input)
{
    uint8_t* ptr = output;

    *(ptr++) = (uint8_t)input;

    return ptr;
}

void LoRa_GenerateJoinFrameIntegrityCode(const uint8_t key[], uint8_t const input[], uint16_t dataLength, uint8_t* output)
{
    AES_CMAC_CTX cmacctx;
    AES_CMAC_Init(&cmacctx);
    AES_CMAC_SetKey(&cmacctx, key);

    AES_CMAC_Update(&cmacctx, input, dataLength);
    uint8_t temp[LORA_AUTHENTICATIONBLOCKBYTES];
    AES_CMAC_Final(temp, &cmacctx);
    memcpy(output, temp, LORA_FRAMEMICBYTES);
}

void GenerateSessionKey(bool generateNetworkKey, const uint8_t* applicationKey, uint32_t networkId, uint32_t applicationNonce, uint16_t deviceNonce, uint8_t* output)
{
    uint8_t input[LORA_ENCRYPTIONBLOCKBYTES];

    input[0] = generateNetworkKey ? 0x01 : 0x02;
    uint8_t* ptr = &input[1];

    ptr = Write3ByteValue(ptr, applicationNonce);
    ptr = Write3ByteValue(ptr, networkId);
    ptr = Write2ByteValue(ptr, deviceNonce);
    memset(ptr, 0, LORA_ENCRYPTIONBLOCKBYTES - (ptr - input));

    aes_context aesContext;
    aes_set_key(applicationKey, LORA_CYPHERKEYBYTES, &aesContext);

    aes_encrypt(input, output, &aesContext);
}

void CryptJoinServer(uint8_t const* key, uint8_t const* input, uint16_t length, uint8_t* output)
{
    aes_context aesContext;
    memset(aesContext.ksch, '\0', 240);
    aes_set_key(key, LORA_CYPHERKEYBYTES, &aesContext);

    aes_decrypt(input, output, &aesContext);
    if (length >= 16) {
        aes_decrypt(input + 16, output + 16, &aesContext);
    }

}

void LoRaWan::SendJoinComplete(uint16_t deviceNonce, uint8_t firstReceiveWindowDataRateoffset, ota_mote_t* mote)
{
    uint8_t tx_buf_len, tx_buf[256];
    uint8_t secondReceiveWindowDataRateNibble = 0;  // unused
    uint8_t networkSessionKey[LORA_CYPHERKEYBYTES];
    uint8_t uncyphered[LORA_MAXDATABYTES];
    uint8_t* current = uncyphered;
    uint32_t applicationNonce = rand() & 0xffffff;  // 24bit

    GenerateSessionKey(true, mote->app_key, network_id, applicationNonce, deviceNonce, networkSessionKey);

    memcpy(mote->network_session_key, networkSessionKey, LORA_CYPHERKEYBYTES);
    
    printf("SendJoinComplete() ");
    if (mote->dev_addr == DEVADDR_NONE) {
        // new mote joining
        printf("new-mote ");
        if ( mote->tx_slot_offset >= (PERIODICITY_SLOTS_ * SLOT_STEPPING)) {
            printf("max motes reached\r\n");
            return;
        }
        mote->dev_addr = ++networkAddress | (network_id << LORA_NETWORKADDRESSBITS);
        mote->tx_slot_offset = next_available_tx_slot;
        next_available_tx_slot += SLOT_STEPPING;
    } else
        printf("rejoin ");

    printf(" mote->dev_addr:%x ", mote->dev_addr);
    printf("networkAddress:%u\r\n", networkAddress);
    *(current++) = MTYPE_JOIN_ACC << 5; // MHDR     0
    current = Write3ByteValue(current, applicationNonce);// 1 2 3
    current = Write3ByteValue(current, network_id);// 4 5 6
    current = Write4ByteValue(current, mote->dev_addr); // 7 8 9 10
    current = Write1ByteValue(current, (firstReceiveWindowDataRateoffset << 4) | (secondReceiveWindowDataRateNibble & 0xf)); // 11 
    //current = Write1ByteValue(current, classARxWindowDelay_s - 1); // 12
    current = Write1ByteValue(current, 0); // 12

    /* put beacon timing answer */
    printf("slots:%u\r\n", rx_slot);
    current = Write2ByteValue(current, rx_slot); // 13, 14
    current = Write2ByteValue(current, mote->tx_slot_offset); // 15, 16
    current = Write2ByteValue(current, (PERIODICITY_SLOTS_ * SLOT_STEPPING)); // 17, 18
    current = Write4ByteValue(current, beaconDur); // 19, 20, 21, 22
    current = Write4ByteValue(current, 0); //
    current = Write2ByteValue(current, 0); //

    uint16_t authenticatedBytes = current - uncyphered;
    LoRa_GenerateJoinFrameIntegrityCode(mote->app_key, uncyphered, authenticatedBytes, current);
    current += LORA_FRAMEMICBYTES;

    tx_buf[0] = MTYPE_JOIN_ACC << 5; // MHDR
    //encrypt
    uint16_t cypherBytes = (current - uncyphered) - LORA_MACHEADERLENGTH;
    CryptJoinServer(mote->app_key, &uncyphered[LORA_MACHEADERLENGTH], cypherBytes, &tx_buf[LORA_MACHEADERLENGTH]);

    /**** RF TX ********/
    tx_buf_len = current - uncyphered;
    if (Radio::LoadTxPacket(tx_buf, tx_buf_len) < 0) {
        printf("\e[33mLoadTxPacket-ja fail\r\n");
    } else
        printf("LoadTxPacket-ja ok\r\n");

    mote->FCntDown = 0;

    GenerateSessionKey(false, mote->app_key, network_id, applicationNonce, deviceNonce, mote->app_session_key);
}

void LoRa_GenerateDataFrameIntegrityCode(const uint8_t key[], uint8_t const input[], uint16_t dataLength, uint32_t address, bool up, uint32_t sequenceNumber, uint8_t* output)
{
    /*
    Generate artificial B[0] block
    Encrypt B[0] to give X[1]

    for n = 1 to number of blocks
        exclusive OR B[n] with X[n] to give Y[n]
        encrypt Yi using key to give X[n+1]
    */
    uint8_t b0[LORA_AUTHENTICATIONBLOCKBYTES];
    memset(b0, 0 , LORA_AUTHENTICATIONBLOCKBYTES);

    b0[ 0] = 0x49; //authentication flags

    b0[ 5] = up ? 0 : 1;
    Write4ByteValue(&b0[6], address);
    Write4ByteValue(&b0[10], sequenceNumber);

    b0[15] = (uint8_t)dataLength;

    AES_CMAC_CTX cmacctx;
    AES_CMAC_Init(&cmacctx);
    AES_CMAC_SetKey(&cmacctx, key);

    AES_CMAC_Update(&cmacctx, b0, LORA_AUTHENTICATIONBLOCKBYTES);
    AES_CMAC_Update(&cmacctx, input, dataLength);

    uint8_t temp[LORA_AUTHENTICATIONBLOCKBYTES];
    AES_CMAC_Final(temp, &cmacctx);

    memcpy(output, temp, LORA_FRAMEMICBYTES);
}

static uint16_t FindBlockOverhang(uint16_t inputDataLength)
{
    return inputDataLength & (LORA_ENCRYPTIONBLOCKBYTES - 1);
}

uint16_t CountBlocks(uint16_t inputDataLength)
{
    uint16_t blockSizeMinus1 = LORA_ENCRYPTIONBLOCKBYTES - 1;
    uint16_t inRoundDown = inputDataLength & ~blockSizeMinus1;
    uint16_t roundUp = (FindBlockOverhang(inputDataLength) > 0) ? 1 : 0;
    uint16_t result = inRoundDown / LORA_ENCRYPTIONBLOCKBYTES + roundUp;

    return result;
}

void BlockExOr(uint8_t const l[], uint8_t const r[], uint8_t out[], uint16_t bytes)
{
    uint8_t const* lptr = l;
    uint8_t const* rptr = r;
    uint8_t* optr = out;
    uint8_t const* const end = out + bytes;

    for (;optr < end; lptr++, rptr++, optr++)
        *optr = *lptr ^ *rptr;
}

void LoRa_EncryptPayload(const uint8_t key[], const uint8_t* in, uint16_t inputDataLength, uint32_t address, bool up, uint32_t sequenceNumber, uint8_t out[])
{
    if (inputDataLength == 0)
        return;

    uint8_t A[LORA_ENCRYPTIONBLOCKBYTES];

    memset(A, 0, LORA_ENCRYPTIONBLOCKBYTES);

    A[ 0] = 0x01; //encryption flags
    A[ 5] = up ? 0 : 1;

    Write4ByteValue(&A[6], address);
    Write4ByteValue(&A[10], sequenceNumber);

    uint16_t const blocks = CountBlocks(inputDataLength);
    uint16_t const overHangBytes = FindBlockOverhang(inputDataLength);

    uint8_t const* blockInput = in;
    uint8_t* blockOutput = out;
    for (uint16_t i = 1; i <= blocks; i++, blockInput += LORA_ENCRYPTIONBLOCKBYTES, blockOutput += LORA_ENCRYPTIONBLOCKBYTES)
    {
        A[15] = (uint8_t)i;

        aes_context aesContext;
        aes_set_key(key, LORA_CYPHERKEYBYTES, &aesContext);

        uint8_t S[LORA_CYPHERKEYBYTES];
        aes_encrypt(A, S, &aesContext);

        uint16_t bytesToExOr;
        if ((i < blocks) || (overHangBytes == 0))
            bytesToExOr = LORA_CYPHERKEYBYTES;
        else
            bytesToExOr = overHangBytes;

        BlockExOr(S, blockInput, blockOutput, bytesToExOr);
    }
}

void put_queue_mac_cmds(ota_mote_t* mote, uint8_t cmd_len, uint8_t* cmd_buf)
{
    int i;
    uint8_t* this_cmd_buf = mote->macCmd_queue[mote->macCmd_queue_in_idx];
    this_cmd_buf[0] = cmd_len;

    printf("put_queue_mac_cmds %u: ", cmd_len);
    for (i = 0; i < cmd_len; i++) {
        this_cmd_buf[i+1] = cmd_buf[i];
        printf("%02x ", cmd_buf[i]);
    }
    printf("\r\n");

    if (++mote->macCmd_queue_in_idx == MAC_CMD_QUEUE_SIZE)
        mote->macCmd_queue_in_idx = 0;

    if (mote->macCmd_queue_in_idx == mote->macCmd_queue_out_idx) {
        printf("macCmd_queue full\r\n");
    }
}

void
LoRaWan::parse_mac_command(ota_mote_t* mote, const uint8_t* rx_cmd_buf, uint8_t rx_cmd_buf_len)
{
    uint8_t cmd_buf[MAC_CMD_SIZE];
    uint8_t rx_cmd_buf_idx = 0;
    int i;
    printf("rx_mac_command(s):");
    for (i = 0; i < rx_cmd_buf_len; i++)
        printf("%02x ", rx_cmd_buf[i]);
    printf("\n");

    while (rx_cmd_buf_idx < rx_cmd_buf_len) {

        switch (rx_cmd_buf[rx_cmd_buf_idx++]) {
            //float diff;
            uint16_t i_diff;
            case MOTE_MAC_LINK_CHECK_REQ:   // 0x02
                printf("MOTE_MAC_LINK_CHECK_REQ\n");
                /* no payload in request */
                cmd_buf[0] = SRV_MAC_LINK_CHECK_ANS;
                cmd_buf[1] = 20;  // db margin above noise floor
                cmd_buf[2] = 1;  // gateway count
                put_queue_mac_cmds(mote, 3, cmd_buf);
                break;
#if 0
#endif
            case MOTE_MAC_BEACON_TIMING_REQ:    // 0x12
                /* no payload in request */
                /*diff = (float)(tick_at_next_beacon - tick_at_RxDone) / 30.0;
                i_diff = (int)floor(diff);*/
                i_diff = rx_slot;
                //printf("MOTE_MAC_BEACON_TIMING_REQ slots:%.1f=%.1fms (int:%u,%u)", diff, diff*30.0, i_diff, i_diff*30);
                printf("MOTE_MAC_BEACON_TIMING_REQ slots:%u", i_diff);
                cmd_buf[0] = SRV_MAC_BEACON_TIMING_ANS;   // 0x12
                cmd_buf[1] = i_diff & 0xff; //lsbyte first byte
                cmd_buf[2] = (i_diff >> 8) & 0xff;
                cmd_buf[3] = 0;   // beacon channel index
                put_queue_mac_cmds(mote, 4, cmd_buf);
                printf("%02x %02x %02x\n", cmd_buf[1], cmd_buf[2], cmd_buf[3]);
                break;
            case MOTE_MAC_PING_SLOT_FREQ_ANS:
                i = rx_cmd_buf[rx_cmd_buf_idx++];
                printf("PING_SLOT_FREQ_ANS status:0x%02x\n", i);
                break;
            case MOTE_MAC_BEACON_FREQ_ANS:
                i = rx_cmd_buf[rx_cmd_buf_idx++];
                printf("BEACON_FREQ_ANS status:0x%02x\n", i);
                break;
            case MOTE_MAC_RX_PARAM_SETUP_ANS:
                i = rx_cmd_buf[rx_cmd_buf_idx++];
                printf("RX_PARAM_SETUP_ANS status:0x%02x\n", i);
                break;
            case MOTE_MAC_NEW_CHANNEL_ANS:
                i = rx_cmd_buf[rx_cmd_buf_idx++];
                printf("NEW_CHANNEL_ANS status:0x%02x\n", i);
                break;
            default:
                printf("[31mTODO mac cmd %02x[0m\n", rx_cmd_buf[rx_cmd_buf_idx-1]);
                return;
        } // ..switch (<mac_command>)
    } // .. while have mac comannds

}

void LoRaWan::classA_downlink(ota_mote_t* mote, uint8_t* txBuf, uint8_t txBufLen)
{
    fhdr_t* fhdr = (fhdr_t*)&txBuf[1];
    uint8_t* mic_ptr;

    fhdr->DevAddr = mote->dev_addr;
    fhdr->FCnt = mote->FCntDown++;
    txBufLen += LORA_MACHEADERLENGTH + sizeof(fhdr_t) + fhdr->FCtrl.dlBits.FOptsLen;
    mic_ptr = &txBuf[txBufLen];

    LoRa_GenerateDataFrameIntegrityCode(mote->network_session_key, txBuf, txBufLen, fhdr->DevAddr, false, fhdr->FCnt, mic_ptr);
    txBufLen += LORA_FRAMEMICBYTES;

    Radio::LoadTxPacket(txBuf, txBufLen);
}

void LoRaWan::parse_uplink(ota_mote_t* mote, uint8_t rx_size, const uint8_t* rx_buf)
{
    uint8_t tx_buf_len, tx_buf[256];
    uint8_t decrypted[256];
    uint32_t calculated_mic, rx_mic;
    fhdr_t *rx_fhdr = (fhdr_t*)&rx_buf[1];
    mhdr_t* rx_mhdr = (mhdr_t*)&rx_buf[0];
    int rxofs = sizeof(mhdr_t) + sizeof(fhdr_t) + rx_fhdr->FCtrl.ulBits.FOptsLen;
    int rxFRMPayload_length = 0;
    const uint8_t* rxFRMPayload = NULL;
    const uint8_t* rx_fport_ptr = NULL;
    bool decrypt_payload = false;

    if ((rx_size - LORA_FRAMEMICBYTES) > rxofs) {
        rxFRMPayload_length = (rx_size - LORA_FRAMEMICBYTES) - (rxofs + 1);
        rxFRMPayload = &rx_buf[rxofs+1];
        rx_fport_ptr = &rx_buf[rxofs];
        filtered_printf(mote->dev_addr, MAC, "port:%d, len:%d", *rx_fport_ptr, rxFRMPayload_length);
    } else {
        filtered_printf(mote->dev_addr, MAC, "no-payload");
    }

    LoRa_GenerateDataFrameIntegrityCode(mote->network_session_key, rx_buf, rx_size-LORA_FRAMEMICBYTES, rx_fhdr->DevAddr, true, rx_fhdr->FCnt, (uint8_t*)&calculated_mic);

    rx_mic = rx_buf[rx_size-1] << 24;
    rx_mic += rx_buf[rx_size-2] << 16;
    rx_mic += rx_buf[rx_size-3] << 8;
    rx_mic += rx_buf[rx_size-4];
    if (calculated_mic != rx_mic) {
        filtered_printf(mote->dev_addr, BOTH, "[31mgenMic:%08lx, rxMic:%08lx\r\n", calculated_mic, rx_mic);
        filtered_printf(mote->dev_addr, BOTH, "mic fail[0m\n");
        return;
    }

    if (rx_fport_ptr != NULL && *rx_fport_ptr == 0) {
        /* mac commands are encrypted onto port 0 */
        LoRa_EncryptPayload(mote->network_session_key, rxFRMPayload, rxFRMPayload_length, rx_fhdr->DevAddr, true, rx_fhdr->FCnt, decrypted);
        filtered_printf(mote->dev_addr, MAC, ", mac commands encrypted on port 0");
        parse_mac_command(mote, decrypted, rxFRMPayload_length);
    } else {
        if (rx_fhdr->FCtrl.ulBits.FOptsLen > 0) {
            /* mac commands are in header */
            filtered_printf(mote->dev_addr, MAC, ", mac commands in header");
            rxofs = sizeof(mhdr_t) + sizeof(fhdr_t);
            parse_mac_command(mote, &rx_buf[rxofs], rx_fhdr->FCtrl.ulBits.FOptsLen);
        }
        if (rxFRMPayload != NULL) {
            decrypt_payload = true;
        }
    }

    fhdr_t* tx_fhdr = (fhdr_t*)&tx_buf[1];
    tx_fhdr->FCtrl.dlBits.FOptsLen = 0;

    /* TODO get queued mac cmds */

    tx_buf_len = 0;

    if (tx_fhdr->FCtrl.dlBits.FOptsLen > 0 || rx_mhdr->bits.MType == MTYPE_CONF_UP ||
        mote->user_downlink_length > 0 || rx_fhdr->FCtrl.ulBits.ADCACKReq)
    {
        /* something to send via downlink */
        if (rx_mhdr->bits.MType == MTYPE_CONF_UP)
            tx_fhdr->FCtrl.dlBits.ACK = 1;
        else
            tx_fhdr->FCtrl.dlBits.ACK = 0;

        if (mote->user_downlink_length > 0) {
            /* add user payload */
            int txo = sizeof(mhdr_t) + sizeof(fhdr_t) + tx_fhdr->FCtrl.dlBits.FOptsLen;
            uint8_t* tx_fport_ptr = &tx_buf[txo];
            uint8_t* txFRMPayload = &tx_buf[txo+1];
            LoRa_EncryptPayload(mote->app_session_key, mote->user_downlink, mote->user_downlink_length, mote->dev_addr, false, mote->FCntDown, txFRMPayload);
            *tx_fport_ptr = mote->user_downlink_port;

            tx_buf_len = tx_fhdr->FCtrl.dlBits.FOptsLen + mote->user_downlink_length + 1; // +1 for fport
            tx_buf[0] = user_dowlink_mtype << 5; // MHDR
            printf(", \e[32mDL-send %d\e[0m", mote->user_downlink_length);
        } else {
            /* downlink not triggered by user_downlink */
            /* downlink triggered by FOpotsLen > 0 or conf_uplink */
            tx_buf[0] = MTYPE_UNCONF_DN << 5; // MHDR
        }

        classA_downlink(mote, tx_buf, tx_buf_len);

        mote->user_downlink_length = 0;  // mark as sent
    }

    if (decrypt_payload) {
        LoRa_EncryptPayload(mote->app_session_key, rxFRMPayload, rxFRMPayload_length, rx_fhdr->DevAddr, true, rx_fhdr->FCnt, decrypted);
        filtered_printf(mote->dev_addr, MAC, ", ");
        decrypted_uplink(mote->dev_eui, mote->dev_addr, decrypted, rxFRMPayload_length, *rx_fport_ptr);
    }

    filtered_printf(mote->dev_addr, BOTH, "\r\n");
}

void LoRaWan::parse_join_req(ota_mote_t* mote, uint8_t rx_size, const uint8_t* rxBuf)
{
    join_req_t* jreq_ptr = (join_req_t*)&rxBuf[0];
    uint32_t rx_mic, calculated_mic;

    LoRa_GenerateJoinFrameIntegrityCode(mote->app_key, rxBuf, rx_size-LORA_FRAMEMICBYTES, (uint8_t*)&calculated_mic);


    rx_mic = rxBuf[rx_size-1] << 24;
    rx_mic += rxBuf[rx_size-2] << 16;
    rx_mic += rxBuf[rx_size-3] << 8;
    rx_mic += rxBuf[rx_size-4];
    if (calculated_mic != rx_mic) {
        printf("join_req mic fail: %08x, %08x\r\n", calculated_mic, rx_mic);
        return;
    }

    /* TODO check devNonce */
    SendJoinComplete(jreq_ptr->DevNonce, 0, mote);
}


int memcmp_rev(const uint8_t* a, const uint8_t* b, uint8_t len)
{
    int a_i, b_i = len - 1;
    for (a_i = 0; a_i < len; a_i++) {
        if (a[a_i] != b[b_i])
            return a[a_i] - b[b_i];
        else
            b_i--;
    }
    return 0;
}

void memcpy_rev(uint8_t* out, uint8_t* in, uint8_t len)
{
    int i;
    out += len;
    for (i = 0; i < len; i++) {
        out--;
        *out = in[i]; 
    }
}

void LoRaWan::init(uint64_t appEui)
{
    srand(time(NULL));

    appEui64 = appEui;
}

uint64_t
eui_buf_to_uint64(const uint8_t* eui)
{
    uint64_t ret = 0;
    int i;

    /* most significant last over air */
    for (i = LORA_EUI_LENGTH - 1; i >= 0; i--) {
        ret <<= 8;
        ret |= eui[i];
    }
    return ret;
}

#ifdef ANY_DEVEUI 
volatile uint8_t num_motes_joined = 0;
#endif /* ANY_DEVEUI */

int LoRaWan::parse_receive(uint8_t rx_size, float rssi, float snr, const uint8_t* rx_buf)
{
    struct _mote_list* my_mote_list;
    int i;
    ota_mote_t* mote = NULL;
    mhdr_t *mhdr = (mhdr_t*)rx_buf;

    if (rx_size <= (sizeof(fhdr_t) + LORA_FRAMEMICBYTES)) {
        printf("too small %d, snr:%.1f %.1fdBm\r\n", rx_size, snr, rssi);
        return 1;
    }
    if (mhdr->bits.major != 0) {
        printf("unsupported major:%u\r\n", mhdr->bits.major);
        return 0;
    }

    if (mhdr->bits.MType == MTYPE_JOIN_REQ) {
        join_req_t* join_req = (join_req_t*)&rx_buf[0];
        printf("MTYPE_JOIN_REQ, ");

        if (eui_buf_to_uint64(join_req->AppEUI) != appEui64) {
            printf("different app_eui\r\n");
            return 0;
        }
#ifdef ANY_DEVEUI
        if (num_motes_joined < N_MOTES && 
            (memcmp_rev(join_req->AppEUI, motes[num_motes_joined].app_eui, LORA_EUI_LENGTH) == 0))
        {
            printf("assigning to mote %u\r\n", num_motes_joined);
            i = num_motes_joined++;
            mote = &motes[i];
            memcpy_rev(mote->dev_eui, join_req->DevEUI, LORA_EUI_LENGTH);
        }
#else
        for (my_mote_list = mote_list; my_mote_list != NULL; my_mote_list = my_mote_list->next) {
            ota_mote_t* _mote = my_mote_list->motePtr;
            if (!_mote)
                continue;

            if ((memcmp_rev(join_req->DevEUI, _mote->dev_eui, LORA_EUI_LENGTH) == 0))
            {
                printf("found mote\r\n");
                mote = _mote;
            }
        }
#endif /* !ANY_DEVEUI */

        if (mote != NULL) {
            printf("Join-Found\r\n");
            parse_join_req(mote, rx_size, rx_buf);
        } else {
            printf("join-not-found:\r\n");
            print_octets_rev("app_eui", join_req->AppEUI, LORA_EUI_LENGTH);
            print_octets_rev("\r\ndev_eui", join_req->DevEUI, LORA_EUI_LENGTH);
            printf("\r\n");
        }
    } else if (mhdr->bits.MType == MTYPE_UNCONF_UP || mhdr->bits.MType == MTYPE_CONF_UP) {
        fhdr_t *fhdr = (fhdr_t*)&rx_buf[1];

        for (my_mote_list = mote_list; my_mote_list != NULL; my_mote_list = my_mote_list->next) {
            ota_mote_t* _mote = my_mote_list->motePtr;
            if (_mote->dev_addr == fhdr->DevAddr) {
                mote = _mote;
            }
        }
        if (mote == NULL) {
            printf("mote %08x not found\r\n", fhdr->DevAddr);
            return 0;
        }

        filtered_printf(mote->dev_addr, MAC, "%u, %lu, %.1fdB, %ddBm, ",
            LoRaWan::rx_slot, time(NULL), snr, rssi
        );

        if (mhdr->bits.MType == MTYPE_UNCONF_UP)
            filtered_printf(mote->dev_addr, MAC, "MTYPE_UNCONF_UP, ");
        else if (mhdr->bits.MType == MTYPE_CONF_UP)
            filtered_printf(mote->dev_addr, MAC, "MTYPE_CONF_UP, ");

        if (mote != NULL) {
            char *ptr, devEuiStr[(LORA_EUI_LENGTH*2)+1];
            for (i = 0, ptr = devEuiStr; i < LORA_EUI_LENGTH; i++) {
                sprintf(ptr, "%02x", mote->dev_eui[i]);
                ptr += 2;
            }
            filtered_printf(mote->dev_addr, MAC, "mote %s / %lx, ", devEuiStr, mote->dev_addr);
            parse_uplink(mote, rx_size, rx_buf);
            filtered_printf(mote->dev_addr, MAC, "\r\n");
        } else {
            printf("mote-not-found %08x\r\n", fhdr->DevAddr);
        }

    } else
        printf(" [31m%02x mtype:%d[0m\r\n", rx_buf[0], mhdr->bits.MType);


    return 0;
}

