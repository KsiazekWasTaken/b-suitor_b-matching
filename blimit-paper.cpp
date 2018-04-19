#include "blimit.hpp"

/*unsigned int bvalue(unsigned int method, unsigned long node_id) {
    switch (method) {
    case 0: return 1;
    default: switch (node_id) {
        case 0: return 2;
        case 1: return 2;
        default: return 1;
        }
    }
}*/

unsigned int bvalue(unsigned int method, unsigned long node_id) {
    switch (method) {
        default: return (2 * node_id + method) % 10;
        case 0: return 4;
        case 1: return 7;
    }
}
