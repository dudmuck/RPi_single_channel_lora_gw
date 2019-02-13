#include <stdint.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <string.h>
#include <fcntl.h>
#include "scgw.h"
#include "commands.h"

char pcbuf[64];
int pcbuf_len;
uint8_t beacon_payload[4];

#define DEFAULT_DOWNLINK_PORT       2

void cmd_hide_mac(uint8_t idx)
{
    scgw_enable_mac_printf(false);
    printf("!show_mac\r\n");
}

void cmd_hide_payload(uint8_t idx)
{
    scgw_enable_app_printf(false);
    printf("!show_app\r\n");
}

void cmd_show_all(uint8_t idx)
{
    scgw_enable_app_printf(true);
    scgw_enable_mac_printf(true);
    printf("show-all\r\n");
}

void
cmd_beacon_payload(uint8_t idx)
{
    uint32_t i;
    uint32_t* ptr;
    sscanf(pcbuf+idx, "%" PRIx32, &i);
    printf("beacon_payload:%08" PRIx32 "\r\n", i);
    ptr = (uint32_t*)beacon_payload;
    *ptr = i;
}

void
cmd_send_downlink(uint8_t idx)
{
    uint8_t buflen, buf[256];
    unsigned int dev_addr;
    sscanf(pcbuf+idx, "%x", &dev_addr);


    while (pcbuf[idx] != ' ') {
        if (pcbuf[++idx] == 0) {
            printf("hit end\r\n");
            return;
        }
    }
    idx++;    // step past space


    buflen = 0;
    for (buflen = 0; pcbuf[idx] > ' '; idx += 2) {
        int o;
        sscanf(pcbuf+idx, "%02x", &o);
        buf[buflen++] = o;
    }

    if (scgw_send_downlink(dev_addr, buf, buflen, DEFAULT_DOWNLINK_PORT) == 0)
        printf("%u bytes scheduled for %" PRIx32 "\r\n", buflen, dev_addr);
    else
        printf("send downlink fail\r\n");
}


void cmd_filter(uint8_t idx)
{
    uint32_t dev_addr_filter;

    if (sscanf(pcbuf+idx, "%x", &dev_addr_filter) != 1) {
        dev_addr_filter = 0;
        printf("filter off\r\n");
    } else
        printf("filtering %x\r\n", dev_addr_filter);

    scgw_set_dev_addr_filter(dev_addr_filter);
}

void cmd_op(uint8_t idx)
{
    int dbm;
    if (sscanf(pcbuf+idx, "%d", &dbm) == 1) {
        scgw_set_tx_dbm(dbm);
        printf("OutputPower:%ddBm\r\n", dbm);
    }
}

void cmd_skip_beacon(uint8_t idx)
{
    unsigned cnt;
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        sscanf(pcbuf+idx, "%u", &cnt);
        scgw_set_beacon_skip(cnt);
    }
    cnt = scgw_get_beacon_skip();
    printf("skip_beacon_cnt:%u\r\n", cnt);
}


void cmd_beacon_endnode_txp(uint8_t idx)
{
    unsigned txi;
    if (sscanf(pcbuf+idx, "%u", &txi) != 1) {
        printf("parse fail\r\n");
        return;
    }

    beacon_payload[0] = CMD_TX_POWER;
    beacon_payload[1] = txi;
    
    printf("txp index:%u\r\n", txi);
}

void cmd_endnode_txp(uint8_t idx)
{
    uint8_t buflen, buf[256];
    unsigned dev_addr, txi;

    if (sscanf(pcbuf+idx, "%x %u", &dev_addr, &txi) != 2) {
        printf("parse fail\r\n");
        return;
    }

    buflen = 0;
    buf[buflen++] = CMD_TX_POWER;
    buf[buflen++] = txi;

    if (scgw_send_downlink(dev_addr, buf, buflen, DEFAULT_DOWNLINK_PORT) == 0)
        printf("txp index %u to mote %" PRIx32 "\r\n", txi, dev_addr);
    else
        printf("send downlink fail\r\n");
}

void cmd_beacon_pwm(uint8_t idx)
{
    unsigned p, d;
    if (sscanf(pcbuf+idx, "%u %u", &p, &d) != 2) {
        printf("parse fail\r\n");
        return;
    }

    beacon_payload[0] = CMD_PWM;
    beacon_payload[1] = p;
    beacon_payload[2] = d;
    
    printf("period:%u duty:%u\r\n", p, d);
}

void cmd_pwm(uint8_t idx)
{
    unsigned dev_addr, p, d;
    uint8_t buflen, buf[256];

    if (sscanf(pcbuf+idx, "%x %u %u", &dev_addr, &p, &d) != 3) {
        printf("parse fail\r\n");
        return;
    }

    buflen = 0;
    buf[buflen++] = CMD_PWM;
    buf[buflen++] = p;
    buf[buflen++] = d;
    
    if (scgw_send_downlink(dev_addr, buf, buflen, DEFAULT_DOWNLINK_PORT) == 0)
        printf("period:%u duty:%u to mote %" PRIx32 "\r\n", p, d, dev_addr);
    else
        printf("send downlink fail\r\n");
}

void cmd_status(uint8_t idx)
{
    scgw_print_status();
}

void cmd_downlink_gpo(uint8_t idx)
{
    uint8_t buflen, buf[256];
    int gpo;
    unsigned int dev_addr;
    sscanf(pcbuf+idx, "%x %d", &dev_addr, &gpo);

    buflen = 0;
    buf[buflen++] = CMD_GPIO_OUT;
    buf[buflen++] = gpo;
    
    if (scgw_send_downlink(dev_addr, buf, buflen, DEFAULT_DOWNLINK_PORT) == 0)
        printf("gpo %d to mote %" PRIx32 "\r\n", gpo, dev_addr);    
    else
        printf("send downlink fail\r\n");
}

void cmd_downlink_rgb(uint8_t idx)
{
    uint8_t buflen, buf[256];
    int r, g ,b;
    unsigned int dev_addr;
    sscanf(pcbuf+idx, "%x %d %d %d", &dev_addr, &r, &g, &b);


    buflen = 0;
    buf[buflen++] = CMD_LED_RGB;
    buf[buflen++] = r;
    buf[buflen++] = g;
    buf[buflen++] = b;
    

    if (scgw_send_downlink(dev_addr, buf, buflen, DEFAULT_DOWNLINK_PORT) == 0)
        printf("rgb %d %d %d to mote %" PRIx32 "\r\n", r, g, b, dev_addr);
    else
        printf("send downlink fail\r\n");
}

void cmd_beacon_gpo(uint8_t idx)
{
    int gpo;
    sscanf(pcbuf+idx, "%d", &gpo);
    beacon_payload[0] = CMD_GPIO_OUT;
    beacon_payload[1] = gpo;   
    printf("beacon gpo: %d\r\n", gpo);
}

void cmd_beacon_rgb(uint8_t idx)
{
    int r, g ,b;
    sscanf(pcbuf+idx, "%d %d %d", &r, &g, &b);
    beacon_payload[0] = CMD_LED_RGB;
    beacon_payload[1] = r;
    beacon_payload[2] = g;
    beacon_payload[3] = b;
    printf("beacon rgb: %d %d %d\r\n", r, g, b);
}

void cmd_help(uint8_t);

typedef struct {
    const char* const cmd;
    void (*handler)(uint8_t args_at);
    const char* const arg_descr;
    const char* const description;
} menu_item_t;

const menu_item_t menu_items[] = 
{   /* after first character, command names must be [A-Za-z] */
    { "?", cmd_help, "","show available commands"}, 
    { ".", cmd_status, "","read status"}, 
    { "f", cmd_filter, "%x","set dev_addr print filter"}, 
    { "b", cmd_beacon_payload, "<%x>","set beacon payload"}, 
    { "sb", cmd_skip_beacon, "<%d>","skip beacons"}, 
    { "list", scgw_cmd_list_motes, "","list active motes"}, 
    { "dl", cmd_send_downlink, "[%x %s]","send downlink <mote-hex-dev-addr> <hex-payload>"}, 
    //{ "rxr", cmd_rx_restart, "", "restart RX"},
    { "brgb", cmd_beacon_rgb, "%u %u %u", "load RGB command into next beacon" },
    { "rgb", cmd_downlink_rgb, "%x %u %u %u", "load RGB command to mote"},
    { "bgpo", cmd_beacon_gpo, "%d", "load output pin command into next beacon"},
    { "gpo", cmd_downlink_gpo, "%x %d", "load output pin command to mote"},
    { "op", cmd_op, "<dBm>","(TX) get/set TX power"},  
    { "p", cmd_pwm, "%x %u %u", "send pwm period, duty to dev_addr"},
    { "bp", cmd_beacon_pwm, "%u %u", "send pwm period, duty on beacon"},
    { "ntxp", cmd_endnode_txp, "%x %u", "send txpower index to dev_addr"},
    { "bntxp", cmd_beacon_endnode_txp, "%u", "send txpower index on beacon"},
    { "hm", cmd_hide_mac, "", "hide mac layer printing "},
    { "hp", cmd_hide_payload, "", "hide payload printing"},
    { "sa", cmd_show_all, "", "show both mac and app layers"},
    { NULL, NULL, NULL, NULL }
};

void cmd_help(uint8_t args_at)
{
    int i;
    
    for (i = 0; menu_items[i].cmd != NULL ; i++) {
        printf("%s%s\t%s\r\n", menu_items[i].cmd, menu_items[i].arg_descr, menu_items[i].description);
    }
    
}

void got_uplink(const uint8_t* devEui, uint32_t dev_addr, const uint8_t* buf, uint8_t buflen, uint8_t port)
{
    if (port == SENSOR_PORT) {
        uint8_t i = 0;
        uint16_t lum = buf[i++] << 8;
        lum += buf[i++];
        app_printf(dev_addr, "SENSOR lum:%u", lum);

        while (i < buflen) {
            uint16_t seq, a_a, a_b, p;
            seq = buf[i++] << 8;
            seq += buf[i++];
            a_a = buf[i++] << 8;
            a_a += buf[i++];
            a_b = buf[i++] << 8;
            a_b += buf[i++];
            p = buf[i++];

            app_printf(dev_addr, ", %u, %u, %u, %u, %u\r\n",
                seq,
                a_a,    /* analog */
                a_b,    /* analog */
                (p & 2) >> 1,    /* digital in */
                p & 1    /* digital out */
            );

        }
    } else {
        int i;
        app_printf(dev_addr, "port%u: ", port);
        for (i = 0; i < buflen; i++)
            app_printf(dev_addr, "%02x ", buf[i]);
    }
}

char rx_isr()
{
    static uint8_t pcbuf_idx = 0;
    static uint8_t prev_len = 0;;
    char c = fgetc(stdin);
    if (c == 0xff)
        return c;

    if (c == 0x7f) {
        if (pcbuf_idx > 0) {
            fputc(8, stdout);
            fputc(' ', stdout);
            fputc(8, stdout);
            pcbuf_idx--;
            fflush(stdout);
        }
    } else if (c == 3) {    // ctrl-C
        pcbuf_len = -1;
    } else if (c == '\r') {
        if (pcbuf_idx == 0) {
            pcbuf_len = prev_len;
        } else {
            pcbuf[pcbuf_idx] = 0;   // null terminate
            prev_len = pcbuf_idx;
            pcbuf_idx = 0;
            pcbuf_len = prev_len;
        }
    } else if (pcbuf_idx < sizeof(pcbuf)) {
        pcbuf[pcbuf_idx++] = c;
        fputc(c, stdout);
        fflush(stdout);
    }

    return c;
}

void
console()
{
    int i;
    uint8_t user_cmd_len;
    
    if (pcbuf_len < 0) {    // ctrl-C
        return;
    }
    if (pcbuf_len == 0)
        return;
        
    printf("\r\n");
        
    /* get end of user-entered command */
    user_cmd_len = 1;   // first character can be any character
    for (i = 1; i <= pcbuf_len; i++) {
        if (pcbuf[i] < 'A' || (pcbuf[i] > 'Z' && pcbuf[i] < 'a') || pcbuf[i] > 'z') {
            user_cmd_len = i;
            break;
        }
    }

    for (i = 0; menu_items[i].cmd != NULL ; i++) {
        int mi_len = strlen(menu_items[i].cmd);

        if (menu_items[i].handler && user_cmd_len == mi_len && (strncmp(pcbuf, menu_items[i].cmd, mi_len) == 0)) {
            while (pcbuf[mi_len] == ' ')   // skip past spaces
                mi_len++;
            menu_items[i].handler(mi_len);
            break;
        }
    }
   
    pcbuf_len = 0;
    printf("> ");
    fflush(stdout); 
}

struct termios orig_termios;

void reset_terminal_mode()
{
    tcsetattr(0, TCSANOW, &orig_termios);
}

void set_conio_terminal_mode()
{
    struct termios new_termios;

    /* take two copies - one for now, one for later */
    tcgetattr(0, &orig_termios);
    memcpy(&new_termios, &orig_termios, sizeof(new_termios));

    /* register cleanup handler, and set the new terminal mode */
    atexit(reset_terminal_mode);
    cfmakeraw(&new_termios);
    tcsetattr(0, TCSANOW, &new_termios);
}

int kbhit()
{
    int ret;
    struct timeval tv = { 0L, (long int)serviceRate_us };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    ret = select(1, &fds, NULL, NULL, &tv);
    return ret;
}

int main(int argc, char* argv[])
{
    if (scgw_start(argv[1], got_uplink) < 0)
        return -1;

    fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);
    set_conio_terminal_mode();

    for (;;) {
        if (kbhit()) {
            char c;
            do {
                c = rx_isr();
            } while (c != 0xff);
        }

        if (pcbuf_len == -1) {
            scgw_stop();
                break;
        }
        console();
        scgw_service();

    } // ..for (;;)

    return 0;
}

