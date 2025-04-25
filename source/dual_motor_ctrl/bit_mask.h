#ifndef BIT_MASK_H
#define BIT_MASK_H

#include <stdint.h>

/*Common use mask and bit enum*/
#ifndef MASK_2BIT
#define MASK_2BIT                 0x3U
#endif

#ifndef MASK_4BIT
#define MASK_4BIT                 0xFU
#endif

#ifndef MASK_7BIT
#define MASK_7BIT                 0x7FU
#endif

#ifndef MASK_8BIT
#define MASK_8BIT                 0xFFU
#endif

#ifndef ERR_8BIT
#define ERR_8BIT                  0xFEU
#endif

#ifndef SNA_8BIT
#define SNA_8BIT                  0xFFU
#endif

#ifndef MASK_12BIT
#define MASK_12BIT                0xFFFU
#endif

#ifndef MASK_16BIT
#define MASK_16BIT                0xFFFFU
#endif

#ifndef ERR_16BIT
#define ERR_16BIT                 0xFEFEU
#endif

#ifndef SNA_16BIT
#define SNA_16BIT                 0xFFFFU
#endif

#ifndef MASK_32BIT
#define MASK_32BIT                0xFFFFFFFFU
#endif

/*Standard physical unit and format use in VDM*/

/*Marco functions*/
#ifndef CLEAR_BIT
#define CLEAR_BIT(a, pos) (a &= ~(0x1 << pos))
#endif

#ifndef SET_BIT
#define SET_BIT(a, pos) (a |= (0x1 << pos))
#endif

#ifndef CLEAR_BITMASK
#define CLEAR_BITMASK(a, mask) (a &= ~(mask))
#endif

#ifndef SET_BITMASK
#define SET_BITMASK(a, mask) (a |= mask)
#endif

#ifndef TEST_BITMASK
#define TEST_BITMASK(a, mask) (a & mask)
#endif

#endif
