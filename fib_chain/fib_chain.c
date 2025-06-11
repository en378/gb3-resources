/* fibchain_bmark.c
 *
 * Deep-dependency test: computes fib(FIBN) in a tight loop ITERS times.
 * LED goes high just before the chain starts, and low immediately after.
 * Measure that high-pulse width on your scope for total execution time.
 */

#include "devscc.h"
#include "sf-types.h"
#include "sh7708.h"
#include "e-types.h"

#define FIBN   40       // depth of the Fibonacci chain
#define ITERS 1000000  // number of iterations

int fib(unsigned int n) {
    if (n < 2) return n;
    unsigned int a = 0, b = 1, c;
    for (unsigned int i = 2; i <= n; i++) {
        c = a + b;
        a = b;
        b = c;
    }
    return b;
}

int main(void) {
    volatile unsigned int *LED = (unsigned int *)0x2000;
    volatile unsigned int acc = 0;

    // toggle LED on → start of fib-chain
    *LED ^= 0xFF;

    // run the deep-dependency workload
    for (int i = 0; i < ITERS; i++) {
        acc += fib(FIBN);
    }

    // use acc in a dummy way to prevent optimizing away
    if (acc == 0xDEADBEEF) {
        *LED ^= 0xFF;
    }


    return 0;
}
