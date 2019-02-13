#ifndef SWIG
#include <stdio.h>
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#endif /* SWIG */

/*! \brief uplink callback
 *         called when uplink received
 * @param dev_addr   device address of end device
 * @param buf        decrypted payload
 * @param buflen     length of decrypted payload
 * @param port
 *  take uplink payload in gateway application
 */
typedef void (*uplink_payload_t)(const uint8_t* devEui, uint32_t dev_addr, const uint8_t* buf, uint8_t buflen, uint8_t port);

/* start single-channel gateway
 * @param confFile  json configuration filename
 * @param callback  function called when uplink received
 */
int scgw_start(const char* confFile, uplink_payload_t callback);

/* \brief stop the gateway
 */
void scgw_stop(void);

/* \brief called from main loop
 */
void scgw_service(void);
/* \brief add end device to network 
 * @param devEuistr Device EUI in ascii hex (8 bytes)
 * @param rootKey   Root AES key in ascii hex (16 bytes)
 */
void scgw_add_mote(const char* devEuistr, const char* rootKey);
void app_printf(uint32_t dev_addr, const char* format, ...);
void scgw_enable_app_printf(bool);
void scgw_enable_mac_printf(bool);

/* \brief schedule downlink to be sent
 * downlink will be sent upon uplink received from device
 * @param dev_addr   device address to send to
 * @param payload    pointer to unencrypted payload to send
 * @param paylen     length of payload
 * @param port       port to send on
 */
int scgw_send_downlink(unsigned dev_addr, const uint8_t* payload, uint8_t paylen, uint8_t port);
int scgw_send_downlink(const uint8_t* devEui, const uint8_t* payload, uint8_t paylen, uint8_t port);

/* \brief filter printing to single end device
 */
void scgw_set_dev_addr_filter(uint32_t);

/* \brief set gateway transmit power
 */
void scgw_set_tx_dbm(int8_t dbm);

/* \brief cause beacons to be unreceiveable
 * for testing end device handling of missing beacon
 * @param n    count of beacons to be skipped
 */
void scgw_set_beacon_skip(uint8_t n);
uint8_t scgw_get_beacon_skip(void);

/* \brief print list of joined end devices
 */
void scgw_cmd_list_motes(uint8_t idx);
void scgw_print_status(void);

extern unsigned serviceRate_us; /**< throttle call-rate to scgw_service() */

