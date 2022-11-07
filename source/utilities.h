/**
 * @file utilities.h
 * @author Mit Bailey (mitbailey@outlook.com)
 * @brief 
 * @version See Git tags for version information.
 * @date 2022.11.07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef UTILITIES_H
#define UTILITIES_H

/**
 * @brief Calculates the integer square root.
 * 
 * @param y The operand.
 * @return uint16_t The square root.
 */
static inline uint16_t isqrt(uint32_t y)
{
    uint64_t L = 0;
    uint64_t M;
    uint64_t R = y + 1;

    while (L != R - 1)
    {
        M = (L + R) / 2; // overflow on 32 bit
        if (M * M <= y) // overflow on 32 bit
        {
            L = M;
        }
        else
        {
            R = M;
        }

    }

    return (uint16_t)L;
}

#endif // UTILITIES_H