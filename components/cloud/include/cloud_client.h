#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Outcome of one request, transport failure and HTTP status collapsed.
 *
 * The caller cannot do anything useful with a status code that it could not
 * also do with one of these, and keeping the mapping in one place is what stops
 * "is 429 a failure?" from being answered differently at two call sites.
 */
typedef enum {
    CLOUD_OK = 0,           // accepted (200, 201, or 429)
    CLOUD_ERR_TRANSPORT,    // never reached the server: DNS, TLS, timeout
    CLOUD_ERR_NOT_REGISTERED, // server rejected this serial number (400)
    CLOUD_ERR_SERVER,       // reached the server, got a status we cannot use
} cloud_result_t;

/**
 * @brief POST one measurement body to the configured endpoint.
 *
 * Blocks for up to CONFIG_CLOUD_HTTP_TIMEOUT_MS. Every call opens and closes
 * its own connection: at one request a minute there is nothing for a kept-alive
 * socket to amortize, and a fresh handle cannot inherit a half-broken TLS
 * session from the previous attempt.
 *
 * @param body      JSON body, NUL-terminated.
 * @param body_len  Length of @p body excluding the terminator.
 */
cloud_result_t cloud_client_post_measures(const char *body, size_t body_len);

/** @brief Short name for a result, for logging. */
const char *cloud_result_name(cloud_result_t result);

#ifdef __cplusplus
}
#endif
