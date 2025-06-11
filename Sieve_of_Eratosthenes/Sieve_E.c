#include "devscc.h"
#include "sf-types.h"
#include "sh7708.h"
#include "e-types.h"

#define MAXN 8192 // ~ Double cache size so every write will miss

int main(void) {
    volatile unsigned int *LED = (unsigned int *)0x2000;
    static unsigned int flags[MAXN+1];
    int p, k;

    // initialize flags[2..MAXN] = true
    for (p = 2; p <= MAXN; p++) {
        flags[p] = 1;
    }

    // toggle LED on → start of sieve
    *LED ^= 0xFF;

    // sieve of Eratosthenes
    for (p = 2; p*p <= MAXN; p++) {
        if (!flags[p]) continue;
        for (k = p*p; k <= MAXN; k += p) {
            flags[k] = 0;
        }
    }

    return 0;
}
