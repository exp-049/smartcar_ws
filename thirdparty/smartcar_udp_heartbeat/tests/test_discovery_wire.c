#include "discovery_wire.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

/* Independent vectors generated with Python struct '<4sQ', '<4sQQQH',
 * '<4sQQQHH', and '<4sQQQQHH'. MAC bytes 01:02:03:04:05:06 form uint48. */
static const uint8_t discover[] = {
    0x53, 0x43, 0x44, 0x31, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11,
};
static const uint8_t subscribe[] = {
    0x53, 0x43, 0x53, 0x31, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11,
    0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01,
    0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE, 0x6E, 0xB2,
};
static const uint8_t release[] = {
    0x53, 0x43, 0x45, 0x31, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11,
    0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01,
    0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE, 0, 0,
};

static void test_requests(void)
{
    discovery_request_t request;
    assert(discovery_wire_request_decode(discover, sizeof(discover), &request));
    assert(request.kind == DISCOVERY_DISCOVER && request.nonce == UINT64_C(0x1122334455667788));
    assert(request.boot_id == 0 && request.session_id == 0 && request.data_port == 0);
    assert(discovery_wire_request_decode(subscribe, sizeof(subscribe), &request));
    assert(request.kind == DISCOVERY_SUBSCRIBE && request.data_port == 45678);
    assert(request.boot_id == UINT64_C(0x0123456789ABCDEF));
    assert(request.session_id == UINT64_C(0xFEDCBA9876543210));
    assert(discovery_wire_request_decode(release, sizeof(release), &request));
    assert(request.kind == DISCOVERY_RELEASE && request.data_port == 0);
    uint8_t changed[DISCOVERY_CONTROL_BYTES + 1];
    memcpy(changed, subscribe, sizeof(subscribe));
    memset(changed + 20, 0, 8);
    assert(!discovery_wire_request_decode(changed, sizeof(subscribe), &request));
    memcpy(changed, subscribe, sizeof(subscribe));
    changed[0] = 'X';
    assert(!discovery_wire_request_decode(changed, sizeof(subscribe), &request));
    for (size_t length = 0; length <= DISCOVERY_CONTROL_BYTES + 1; ++length) {
        if (length != sizeof(subscribe)) {
            assert(!discovery_wire_request_decode(subscribe, length, &request));
        }
        if (length != sizeof(discover)) {
            assert(!discovery_wire_request_decode(discover, length, &request));
        }
    }
    assert(!discovery_wire_request_decode(NULL, sizeof(discover), &request));
    assert(!discovery_wire_request_decode(discover, sizeof(discover), NULL));
    memcpy(changed, subscribe, sizeof(subscribe));
    changed[28] = 0xFF;
    changed[29] = 0xFF;
    assert(discovery_wire_request_decode(changed, sizeof(subscribe), &request));
    assert(request.data_port == UINT16_MAX);
    /* Malformed request must not partially overwrite the caller's request. */
    discovery_request_t previous = request;
    assert(!discovery_wire_request_decode(changed, sizeof(changed), &request));
    assert(memcmp(&previous, &request, sizeof(request)) == 0);
    uint64_t boot = 0;
    assert(discovery_wire_boot_id_value("0123456789abcdef", &boot));
    assert(boot == UINT64_C(0x0123456789ABCDEF));
    assert(!discovery_wire_boot_id_value("0123456789ABCDEF", &boot));
    assert(!discovery_wire_boot_id_value("0123456789abcde", &boot));
    assert(!discovery_wire_boot_id_value("0123456789abcdeg", &boot));
    assert(!discovery_wire_boot_id_value(NULL, &boot));
    assert(!discovery_wire_boot_id_value("0123456789abcdef", NULL));
}

static void test_responses(void)
{
    const uint64_t nonce = UINT64_C(0x1122334455667788);
    const uint64_t device = UINT64_C(0x010203040506);
    const uint64_t boot = UINT64_C(0x0123456789ABCDEF);
    const uint64_t session = UINT64_C(0xFEDCBA9876543210);
    const uint8_t expected_announce[] = {
        0x53, 0x43, 0x41, 0x31, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11,
        0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0, 0,
        0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01, 0xB9, 0x22, 0x03, 0,
    };
    const uint8_t expected_ack[] = {
        0x53, 0x43, 0x4B, 0x31, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11,
        0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0, 0,
        0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01,
        0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE, 0x70, 0x17, 0, 0,
    };
    assert(sizeof(expected_announce) == DISCOVERY_ANNOUNCE_BYTES);
    assert(sizeof(expected_ack) == DISCOVERY_ACK_BYTES);
    uint8_t output[DISCOVERY_ACK_BYTES + 1];
    memset(output, 0x5A, sizeof(output));
    assert(discovery_wire_announce_encode(output, DISCOVERY_ANNOUNCE_BYTES,
                                          nonce, device, boot, 8889, 3) == DISCOVERY_ANNOUNCE_BYTES);
    assert(memcmp(output, expected_announce, sizeof(expected_announce)) == 0);
    assert(output[DISCOVERY_ANNOUNCE_BYTES] == 0x5A);
    assert(discovery_wire_ack_encode(output, DISCOVERY_ACK_BYTES,
                                     nonce, device, boot, session, 6000, DISCOVERY_STATUS_OK) == DISCOVERY_ACK_BYTES);
    assert(memcmp(output, expected_ack, sizeof(expected_ack)) == 0);
    assert(output[DISCOVERY_ACK_BYTES] == 0x5A);
    for (unsigned status = 0; status <= 3; ++status) {
        assert(discovery_wire_ack_encode(output, sizeof(output), nonce, device, boot,
                                         session, 0, (discovery_status_t)status) == DISCOVERY_ACK_BYTES);
        assert(output[36] == 0 && output[37] == 0 && output[38] == status && output[39] == 0);
    }
    assert(discovery_wire_ack_encode(output, sizeof(output), nonce, device, boot,
                                     session, 6000, DISCOVERY_STATUS_BUSY) == -1);
    assert(discovery_wire_ack_encode(output, sizeof(output), nonce, device, boot,
                                     session, 1, DISCOVERY_STATUS_OK) == -1);
    assert(discovery_wire_ack_encode(output, sizeof(output), nonce, device, boot,
                                     session, 0, (discovery_status_t)4) == -1);
    assert(discovery_wire_ack_encode(output, sizeof(output), nonce, device, boot,
                                     session, 0, (discovery_status_t)-1) == -1);
    assert(discovery_wire_ack_encode(output, sizeof(output), nonce, device, boot,
                                     0, 0, DISCOVERY_STATUS_OK) == -1);
    assert(discovery_wire_ack_encode(output, sizeof(output), nonce, UINT64_MAX, boot,
                                     session, 0, DISCOVERY_STATUS_OK) == -1);
    assert(discovery_wire_announce_encode(output, sizeof(output), nonce, device, boot, 0, 3) == -1);
    assert(discovery_wire_announce_encode(output, sizeof(output), nonce, device, boot, 8889, 4) == -1);
    assert(discovery_wire_announce_encode(output, sizeof(output), nonce, UINT64_MAX, boot, 8889, 3) == -1);
    for (size_t capacity = 0; capacity < DISCOVERY_ACK_BYTES; ++capacity) {
        assert(discovery_wire_ack_encode(output, capacity, nonce, device, boot,
                                         session, 6000, DISCOVERY_STATUS_OK) == -1);
        if (capacity < DISCOVERY_ANNOUNCE_BYTES) {
            assert(discovery_wire_announce_encode(output, capacity, nonce, device, boot, 8889, 3) == -1);
        }
    }
    assert(discovery_wire_announce_encode(NULL, sizeof(output), nonce, device, boot, 8889, 3) == -1);
    assert(discovery_wire_ack_encode(NULL, sizeof(output), nonce, device, boot,
                                     session, 6000, DISCOVERY_STATUS_OK) == -1);
}

static void test_lease(void)
{
    const uint32_t ip_a = UINT32_C(0x0100000A);
    const uint32_t ip_b = UINT32_C(0x0200000A);
    discovery_lease_t lease;
    discovery_lease_init(&lease);
    assert(!discovery_lease_current(&lease, 0) && lease.generation == 1);
    assert(discovery_lease_claim(&lease, ip_a, 100, 8888, 1000) == DISCOVERY_STATUS_OK);
    assert(lease.active && lease.source_ipv4 == ip_a && lease.session_id == 100 && lease.data_port == 8888);
    assert(lease.expires_us == 6001000);
    uint32_t first_generation = lease.generation;
    assert(first_generation != 1);
    assert(discovery_lease_claim(&lease, ip_a, 100, 8888, 1001000) == DISCOVERY_STATUS_OK);
    assert(lease.expires_us == 7001000 && lease.generation == first_generation);
    int64_t deadline = lease.expires_us;
    assert(discovery_lease_claim(&lease, ip_b, 100, 8888, 1002000) == DISCOVERY_STATUS_BUSY);
    assert(discovery_lease_claim(&lease, ip_a, 101, 8888, 1002000) == DISCOVERY_STATUS_BUSY);
    assert(lease.generation == first_generation && lease.expires_us == deadline);
    assert(discovery_lease_release(&lease, ip_a, 101, 1002000) == DISCOVERY_STATUS_BUSY);
    assert(discovery_lease_release(&lease, ip_b, 100, 1002000) == DISCOVERY_STATUS_BUSY);
    assert(discovery_lease_claim(&lease, ip_a, 100, 1023, 1002000) == DISCOVERY_STATUS_BAD_PORT);
    assert(discovery_lease_claim(&lease, ip_a, 0, 8888, 1002000) == DISCOVERY_STATUS_BUSY);
    assert(discovery_lease_claim(&lease, ip_a, 100, 8888, -1) == DISCOVERY_STATUS_BUSY);
    assert(discovery_lease_claim(&lease, ip_a, 100, 8888, INT64_MAX) == DISCOVERY_STATUS_BUSY);
    assert(lease.generation == first_generation && lease.expires_us == deadline);
    assert(discovery_lease_claim(&lease, ip_a, 100, UINT16_MAX, 1003000) == DISCOVERY_STATUS_OK);
    assert(lease.data_port == UINT16_MAX && lease.generation != first_generation);
    uint32_t changed_port_generation = lease.generation;
    assert(discovery_lease_release(&lease, ip_a, 100, 1004000) == DISCOVERY_STATUS_OK);
    assert(!lease.active && lease.generation != changed_port_generation);
    assert(discovery_lease_release(&lease, ip_a, 100, 1004001) == DISCOVERY_STATUS_BUSY);
    assert(discovery_lease_claim(&lease, ip_b, 200, 1024, 2000000) == DISCOVERY_STATUS_OK);
    deadline = lease.expires_us;
    uint32_t before_expiry = lease.generation;
    assert(discovery_lease_current(&lease, deadline - 1));
    assert(!discovery_lease_current(&lease, deadline));
    assert(lease.generation != before_expiry);
    uint32_t after_expiry = lease.generation;
    assert(!discovery_lease_current(&lease, deadline + 1) && lease.generation == after_expiry);
    assert(discovery_lease_claim(&lease, ip_a, 300, 8888, deadline + 1) == DISCOVERY_STATUS_OK);
    assert(lease.source_ipv4 == ip_a && lease.session_id == 300 && lease.generation != after_expiry);
    discovery_lease_clear(&lease);
    assert(!lease.active && lease.source_ipv4 == 0 && lease.session_id == 0 && lease.data_port == 0);
    lease.generation = UINT32_MAX;
    assert(discovery_lease_claim(&lease, ip_a, 400, 8888, 0) == DISCOVERY_STATUS_OK);
    assert(lease.generation == 1);  /* Generation zero stays reserved. */
    assert(!discovery_lease_current(&lease, -1) && lease.generation == 2);
    discovery_lease_init(NULL);
    discovery_lease_clear(NULL);
    assert(!discovery_lease_expire(NULL, 0));
    assert(!discovery_lease_current(NULL, 0));
    assert(discovery_lease_claim(NULL, ip_a, 1, 8888, 0) == DISCOVERY_STATUS_BUSY);
    assert(discovery_lease_release(NULL, ip_a, 1, 0) == DISCOVERY_STATUS_BUSY);
}

int main(void)
{
    test_requests();
    test_responses();
    test_lease();
    puts("PASS: discovery LE vectors/exact lengths; ACK 40 bytes; exclusive lease/renew/release/expiry/generation bounds");
    return 0;
}
