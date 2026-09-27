#include "foc_shell_parse.h"

#include <assert.h>
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

static void expect_i32(const char *text, int32_t expected)
{
    int32_t actual = 17;
    assert(foc_shell_parse_i32(text, &actual) == 1U);
    assert(actual == expected);
}

static void expect_i32_rejected(const char *text)
{
    int32_t actual = 0x12345678;
    assert(foc_shell_parse_i32(text, &actual) == 0U);
    assert(actual == 0x12345678);
}

static void expect_u32(const char *text, uint32_t expected)
{
    uint32_t actual = 17U;
    assert(foc_shell_parse_u32(text, &actual) == 1U);
    assert(actual == expected);
}

static void expect_u32_rejected(const char *text)
{
    uint32_t actual = 0xA5A5A5A5U;
    assert(foc_shell_parse_u32(text, &actual) == 0U);
    assert(actual == 0xA5A5A5A5U);
}

static void test_signed_decimal_boundaries(void)
{
    expect_i32("0", 0);
    expect_i32("-0", 0);
    expect_i32("+0", 0);
    expect_i32("0012", 12);
    expect_i32("+582", 582);
    expect_i32("-582", -582);
    expect_i32("2147483647", INT32_MAX);
    expect_i32("-2147483648", INT32_MIN);

    expect_i32_rejected(0);
    expect_i32_rejected("");
    expect_i32_rejected("+");
    expect_i32_rejected("-");
    expect_i32_rejected("2147483648");
    expect_i32_rejected("-2147483649");
}

static void test_unsigned_decimal_boundaries(void)
{
    expect_u32("0", 0U);
    expect_u32("00042", 42U);
    expect_u32("4294967295", UINT32_MAX);

    expect_u32_rejected(0);
    expect_u32_rejected("");
    expect_u32_rejected("+1");
    expect_u32_rejected("-0");
    expect_u32_rejected("4294967296");
}

static void test_non_decimal_and_partial_tokens_are_rejected(void)
{
    static const char *const invalid_tokens[] = {
        " 1", "1 ", "1.0", ".5", "1e3", "0x10", "nan", "NaN",
        "inf", "-inf", "1rpm", "1,2", "++1", "--1", "+-1", "-+1",
    };
    size_t index;

    for (index = 0U;
         index < (sizeof(invalid_tokens) / sizeof(invalid_tokens[0]));
         ++index)
    {
        expect_i32_rejected(invalid_tokens[index]);
        expect_u32_rejected(invalid_tokens[index]);
    }
}

static void test_scaled_units_and_failure_preserve_output(void)
{
    float actual = 99.0f;

    assert(foc_shell_parse_i32_scaled("1150", 1000U, &actual) == 1U);
    assert(actual == 1.15f);
    assert(foc_shell_parse_i32_scaled("-125", 1000U, &actual) == 1U);
    assert(actual == -0.125f);
    assert(foc_shell_parse_i32_scaled("582", 1U, &actual) == 1U);
    assert(actual == 582.0f);

    actual = 99.0f;
    assert(foc_shell_parse_i32_scaled("1.15", 1000U, &actual) == 0U);
    assert(actual == 99.0f);
    assert(foc_shell_parse_i32_scaled("1", 0U, &actual) == 0U);
    assert(actual == 99.0f);
    assert(foc_shell_parse_i32_scaled("1", 1000U, 0) == 0U);
}

int main(void)
{
    test_signed_decimal_boundaries();
    test_unsigned_decimal_boundaries();
    test_non_decimal_and_partial_tokens_are_rejected();
    test_scaled_units_and_failure_preserve_output();
    puts("FOC SHELL PARSE: PASS");
    return 0;
}
