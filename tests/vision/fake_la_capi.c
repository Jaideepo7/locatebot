/* Fake liblocate_anything.so with the la_capi.h ABI (v1), for testing the ctypes binding
 * without the 4.7 GB model. Behaviour:
 *   - la_capi_load fails (NULL) when the path contains "missing".
 *   - la_capi_locate_buffer fails (NULL, last_error set) when the prompt contains "FAIL".
 *   - otherwise it returns one detection labelled with the prompt, box = [len, mode, first_byte, 99].
 * Counters let tests check that every returned string and context is freed. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct la_ctx { char last_error[128]; } la_ctx;

int fake_abi = 1;
int fake_live_ctx = 0;
int fake_live_strings = 0;
int fake_last_threads = -1;

int la_capi_abi_version(void) { return fake_abi; }

la_ctx* la_capi_load(const char* path, int n_threads) {
    if (!path || strstr(path, "missing")) return NULL;
    la_ctx* c = calloc(1, sizeof *c);
    fake_live_ctx++;
    fake_last_threads = n_threads;
    return c;
}

void la_capi_free(la_ctx* c) { if (c) { free(c); fake_live_ctx--; } }

char* la_capi_locate_buffer(la_ctx* c, const unsigned char* bytes, size_t len, const char* prompt, int mode) {
    if (!c) return NULL;
    if (strstr(prompt, "FAIL")) { snprintf(c->last_error, sizeof c->last_error, "fake failure"); return NULL; }
    c->last_error[0] = '\0';
    char* out = malloc(512);
    snprintf(out, 512, "{\"detections\":[{\"label\":\"%s\",\"box\":[%zu,%d,%d,99]}]}",
             prompt, len, mode, len ? bytes[0] : -1);
    fake_live_strings++;
    return out;
}

void la_capi_free_string(char* s) { if (s) { free(s); fake_live_strings--; } }

const char* la_capi_last_error(la_ctx* c) { return c ? c->last_error : ""; }
