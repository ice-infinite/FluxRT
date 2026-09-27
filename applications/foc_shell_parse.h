#ifndef FOC_SHELL_PARSE_H
#define FOC_SHELL_PARSE_H

/*
 * FluxRT - allocation-free strict decimal parsing for application/Shell code.
 *
 * The parser accepts one complete ASCII decimal token.  It never skips
 * whitespace and never accepts fractions, exponents, NaN/Inf, hexadecimal or
 * trailing characters.  Output values are left unchanged on failure.
 */

#include <stdint.h>

uint32_t foc_shell_parse_i32(const char *text, int32_t *value);
uint32_t foc_shell_parse_u32(const char *text, uint32_t *value);

/* Parse a signed decimal integer and convert it to a float in base units:
 *     value = parsed_integer / denominator
 * denominator must be non-zero.  Shell milli-units therefore use 1000U. */
uint32_t foc_shell_parse_i32_scaled(const char *text,
                                    uint32_t denominator,
                                    float *value);

#endif /* FOC_SHELL_PARSE_H */
