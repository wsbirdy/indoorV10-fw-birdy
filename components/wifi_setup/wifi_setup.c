#include <stdio.h>
#include <string.h>

#include "wifi_setup.h"

#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "wifi_sta.h"

static const char *TAG = "wifi_setup";

#define AP_CHANNEL          1
#define AP_MAX_CONN         2

/*
 * How long the confirmation page has to reach the browser before the AP goes
 * away underneath it. Tearing down the SoftAP drops the client, so the response
 * has to be fully flushed first; a couple of seconds is generous for one small
 * page over a link with a single station on it.
 */
#define CLOSE_DELAY_MS      2500

/* Longest form body worth reading: SSID and passphrase are 32 and 64 bytes. */
#define FORM_BODY_MAX       256

static httpd_handle_t     s_server;
static esp_netif_t       *s_ap_netif;
static esp_timer_handle_t s_close_timer;
static char               s_ap_ssid[33];
static bool               s_active;
static bool               s_received;


/* -------------------------------------------------------------- the page */

/*
 * No network scan. Scanning from AP+STA takes the radio off-channel for a few
 * seconds, which drops the very phone that is filling in the form -- a list of
 * nearby SSIDs is not worth losing the client mid-setup. The user types the
 * name instead.
 */
static const char PAGE_FORM[] =
    "<!doctype html><html><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>Wi-Fi setup</title><style>"
    "body{font-family:system-ui,sans-serif;margin:0;padding:24px;background:#f4f4f5;color:#18181b}"
    "main{max-width:22rem;margin:0 auto;background:#fff;padding:24px;border-radius:12px;"
    "box-shadow:0 1px 3px rgba(0,0,0,.1)}"
    "h1{font-size:1.25rem;margin:0 0 4px}p{color:#52525b;font-size:.875rem;margin:0 0 20px}"
    "label{display:block;font-size:.875rem;font-weight:600;margin:16px 0 6px}"
    "input{width:100%%;box-sizing:border-box;padding:10px;font-size:1rem;"
    "border:1px solid #d4d4d8;border-radius:8px}"
    "button{width:100%%;margin-top:24px;padding:12px;font-size:1rem;font-weight:600;"
    "color:#fff;background:#2563eb;border:0;border-radius:8px}"
    "</style></head><body><main>"
    "<h1>Wi-Fi setup</h1>"
    "<p>%s</p>"
    "<form method=\"POST\" action=\"/save\">"
    "<label for=\"s\">Network name</label>"
    "<input id=\"s\" name=\"ssid\" maxlength=\"32\" required autocapitalize=\"off\""
    " autocorrect=\"off\" spellcheck=\"false\">"
    "<label for=\"p\">Password</label>"
    "<input id=\"p\" name=\"pass\" type=\"password\" maxlength=\"64\">"
    "<button type=\"submit\">Save and connect</button>"
    "</form></main></body></html>";

static const char PAGE_SAVED[] =
    "<!doctype html><html><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>Saved</title><style>"
    "body{font-family:system-ui,sans-serif;margin:0;padding:24px;background:#f4f4f5;color:#18181b}"
    "main{max-width:22rem;margin:0 auto;background:#fff;padding:24px;border-radius:12px;"
    "box-shadow:0 1px 3px rgba(0,0,0,.1)}"
    "h1{font-size:1.25rem;margin:0 0 8px}p{color:#52525b;font-size:.875rem;margin:0 0 8px}"
    "</style></head><body><main>"
    "<h1>Saved</h1>"
    "<p>Connecting to <b>%s</b>.</p>"
    "<p>This hotspot is closing. Check the monitor's screen: it shows the new "
    "network once the connection succeeds, or the reason it failed.</p>"
    "</main></body></html>";


/* ------------------------------------------------------------ form parsing */

static int hex_value(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/**
 * @brief Copy one application/x-www-form-urlencoded field out of @p body.
 *
 * Decodes in the same pass rather than in place: the body buffer is shared by
 * both fields, and a decode that shortened it would move the second field.
 */
static bool form_field(const char *body, const char *name, char *out, size_t out_size)
{
    const size_t name_len = strlen(name);
    const char *p = body;

    while (*p) {
        const bool at_field = (strncmp(p, name, name_len) == 0) && (p[name_len] == '=');
        if (at_field) {
            p += name_len + 1;
            size_t n = 0;

            while (*p && *p != '&' && n + 1 < out_size) {
                if (*p == '+') {
                    out[n++] = ' ';
                    p++;
                } else if (*p == '%' && hex_value(p[1]) >= 0 && hex_value(p[2]) >= 0) {
                    out[n++] = (char)((hex_value(p[1]) << 4) | hex_value(p[2]));
                    p += 3;
                } else {
                    out[n++] = *p++;
                }
            }
            out[n] = '\0';
            return true;
        }

        // Advance to just past the next separator.
        const char *amp = strchr(p, '&');
        if (!amp) {
            return false;
        }
        p = amp + 1;
    }
    return false;
}


/* ------------------------------------------------------------- handlers */

static esp_err_t form_get_handler(httpd_req_t *req)
{
    char page[sizeof(PAGE_FORM) + 128];
    char intro[96];

    snprintf(intro, sizeof(intro), "Enter the network for this air monitor to join.");
    const int n = snprintf(page, sizeof(page), PAGE_FORM, intro);

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, page, (n > 0 && (size_t)n < sizeof(page)) ? n : HTTPD_RESP_USE_STRLEN);
}

static esp_err_t save_post_handler(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len >= FORM_BODY_MAX) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad form length");
        return ESP_FAIL;
    }

    char body[FORM_BODY_MAX];
    int received = 0;
    while (received < req->content_len) {
        const int n = httpd_req_recv(req, body + received, req->content_len - received);
        if (n <= 0) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "truncated form");
            return ESP_FAIL;
        }
        received += n;
    }
    body[received] = '\0';

    char ssid[33] = { 0 };
    char pass[65] = { 0 };
    if (!form_field(body, "ssid", ssid, sizeof(ssid)) || ssid[0] == '\0') {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "network name is required");
        return ESP_FAIL;
    }
    form_field(body, "pass", pass, sizeof(pass));   // absent means an open network

    ESP_LOGI(TAG, "form submitted for \"%s\" (%u character password)",
             ssid, (unsigned)strlen(pass));

    const esp_err_t err = wifi_sta_set_credentials(ssid, pass);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "could not store credentials: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "could not store credentials");
        return ESP_FAIL;
    }

    char page[sizeof(PAGE_SAVED) + 64];
    const int n = snprintf(page, sizeof(page), PAGE_SAVED, ssid);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, page, (n > 0 && (size_t)n < sizeof(page)) ? n : HTTPD_RESP_USE_STRLEN);

    /*
     * The teardown cannot happen here: httpd_stop() from inside a handler waits
     * on the worker running that handler. Hand it to the timer task instead,
     * far enough out that the page above has reached the browser.
     */
    s_received = true;
    esp_timer_stop(s_close_timer);
    esp_timer_start_once(s_close_timer, (uint64_t)CLOSE_DELAY_MS * 1000);
    return ESP_OK;
}

/*
 * Phones probe a handful of well-known URLs to decide whether a network has a
 * captive portal. Redirecting every unknown path to the form turns that probe
 * into the setup page popping up on its own, which saves the user having to
 * type an IP address.
 */
static esp_err_t redirect_handler(httpd_req_t *req, httpd_err_code_t err)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", WIFI_SETUP_PORTAL_URL "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}


/* ------------------------------------------------------------- lifecycle */

static void close_timer_cb(void *arg)
{
    ESP_LOGI(TAG, "closing the setup portal");
    wifi_setup_stop();
}

static esp_err_t start_ap(void)
{
    if (!s_ap_netif) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
        ESP_RETURN_ON_FALSE(s_ap_netif, ESP_ERR_NO_MEM, TAG, "ap netif create failed");
    }

    // Serial suffix keeps two monitors in the same room distinguishable.
    const char *serial = wifi_sta_serial();
    const size_t serial_len = strlen(serial);
    snprintf(s_ap_ssid, sizeof(s_ap_ssid), "AirGradient-Setup-%s",
             serial_len >= 4 ? serial + serial_len - 4 : "0000");

    wifi_config_t ap_cfg = {
        .ap = {
            .channel = AP_CHANNEL,
            .max_connection = AP_MAX_CONN,
            .authmode = WIFI_AUTH_OPEN,
        },
    };
    strlcpy((char *)ap_cfg.ap.ssid, s_ap_ssid, sizeof(ap_cfg.ap.ssid));
    ap_cfg.ap.ssid_len = (uint8_t)strlen(s_ap_ssid);

    // AP+STA, not AP: the station keeps retrying underneath, so a network that
    // returns on its own heals without anybody completing the form.
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_APSTA), TAG, "set mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg), TAG, "ap config failed");
    return ESP_OK;
}

static esp_err_t start_server(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.lru_purge_enable = true;
    cfg.max_uri_handlers = 4;

    ESP_RETURN_ON_ERROR(httpd_start(&s_server, &cfg), TAG, "httpd start failed");

    const httpd_uri_t form_uri = {
        .uri = "/", .method = HTTP_GET, .handler = form_get_handler,
    };
    const httpd_uri_t save_uri = {
        .uri = "/save", .method = HTTP_POST, .handler = save_post_handler,
    };
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_server, &form_uri), TAG, "GET / failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_server, &save_uri), TAG, "POST /save failed");
    ESP_RETURN_ON_ERROR(httpd_register_err_handler(s_server, HTTPD_404_NOT_FOUND, redirect_handler),
                        TAG, "404 handler failed");
    return ESP_OK;
}

esp_err_t wifi_setup_start(void)
{
    if (s_active) {
        return ESP_OK;
    }

    if (!s_close_timer) {
        const esp_timer_create_args_t args = {
            .callback = close_timer_cb,
            .name = "wifi_setup_close",
        };
        ESP_RETURN_ON_ERROR(esp_timer_create(&args, &s_close_timer), TAG, "timer create failed");
    }

    s_received = false;

    ESP_RETURN_ON_ERROR(start_ap(), TAG, "softap failed");

    const esp_err_t err = start_server();
    if (err != ESP_OK) {
        // Do not leave an AP up with nothing serving on it.
        esp_wifi_set_mode(WIFI_MODE_STA);
        return err;
    }

    s_active = true;

    // Bounded on its own so an abandoned setup does not leave the radio in
    // AP+STA for the rest of the device's uptime.
    esp_timer_stop(s_close_timer);
    esp_timer_start_once(s_close_timer, (uint64_t)WIFI_SETUP_TIMEOUT_MS * 1000);

    ESP_LOGI(TAG, "portal up: join \"%s\" (open) and browse to %s",
             s_ap_ssid, WIFI_SETUP_PORTAL_URL);
    return ESP_OK;
}

void wifi_setup_stop(void)
{
    if (!s_active) {
        return;
    }

    esp_timer_stop(s_close_timer);

    if (s_server) {
        httpd_stop(s_server);
        s_server = NULL;
    }

    // Back to station-only. The netif is left allocated: creating it is the
    // expensive half and a second setup round would only have to make it again.
    esp_wifi_set_mode(WIFI_MODE_STA);

    s_active = false;
    s_ap_ssid[0] = '\0';
    ESP_LOGI(TAG, "portal down, radio back to station-only");
}

bool wifi_setup_is_active(void)
{
    return s_active;
}

const char *wifi_setup_ap_ssid(void)
{
    return s_ap_ssid;
}

bool wifi_setup_credentials_received(void)
{
    return s_received;
}
