#include "foc_shell_parse.h"

#include <limits.h>

static uint32_t foc_shell_parse_magnitude(const char *text,
                                          uint32_t limit,
                                          uint32_t *magnitude)
{
    uint32_t accumulator = 0U;
    const char *cursor = text;

    if ((text == 0) || (magnitude == 0) || (*text == '\0'))
    {
        return 0U;
    }

    while (*cursor != '\0')
    {
        uint32_t digit;

        if ((*cursor < '0') || (*cursor > '9'))
        {
            return 0U;
        }
        digit = (uint32_t)(*cursor - '0');
        if (accumulator > ((limit - digit) / 10U))
        {
            return 0U;
        }
        accumulator = (accumulator * 10U) + digit;
        ++cursor;
    }

    *magnitude = accumulator;
    return 1U;
}

uint32_t foc_shell_parse_u32(const char *text, uint32_t *value)
{
    uint32_t parsed;

    if ((value == 0) ||
        (foc_shell_parse_magnitude(text, UINT32_MAX, &parsed) == 0U))
    {
        return 0U;
    }
    *value = parsed;
    return 1U;
}

uint32_t foc_shell_parse_i32(const char *text, int32_t *value)
{
    const char *magnitude_text = text;
    uint32_t negative = 0U;
    uint32_t limit = (uint32_t)INT32_MAX;
    uint32_t magnitude;
    int32_t parsed;

    if ((text == 0) || (value == 0) || (*text == '\0'))
    {
        return 0U;
    }
    if ((*magnitude_text == '-') || (*magnitude_text == '+'))
    {
        negative = (*magnitude_text == '-') ? 1U : 0U;
        ++magnitude_text;
        if (*magnitude_text == '\0')
        {
            return 0U;
        }
    }
    if (negative != 0U)
    {
        limit = (uint32_t)INT32_MAX + 1U;
    }
    if (foc_shell_parse_magnitude(magnitude_text, limit, &magnitude) == 0U)
    {
        return 0U;
    }

    if ((negative != 0U) && (magnitude == ((uint32_t)INT32_MAX + 1U)))
    {
        parsed = INT32_MIN;
    }
    else
    {
        parsed = (int32_t)magnitude;
        if (negative != 0U)
        {
            parsed = -parsed;
        }
    }
    *value = parsed;
    return 1U;
}

uint32_t foc_shell_parse_i32_scaled(const char *text,
                                    uint32_t denominator,
                                    float *value)
{
    int32_t parsed;
    float converted;

    if ((value == 0) || (denominator == 0U) ||
        (foc_shell_parse_i32(text, &parsed) == 0U))
    {
        return 0U;
    }
    converted = (float)parsed / (float)denominator;
    *value = converted;
    return 1U;
}
