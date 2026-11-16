def main():
    print("Hello from python!")

    while True:
        user_input = input("Please enter a number to convert to fixed point (or type 'exit' to quit): \n")
        if user_input.strip().lower() == 'exit':
            break
        try:
            if user_input.endswith('/2'):
                user_input = user_input[:-2]  # Remove the '/2' suffix
                n = float(user_input) / 2
            else:
                n = float(user_input)
        except ValueError:
            print("Please enter a valid number.")
            continue

        # convert_to_fixed_point(n, 9, 23)
        # convert_to_fixed_point(n, 1, 31)
        convert_to_fixed_point(n, 5, 27)


def convert_to_fixed_point(number, integer_bits, fractional_bits):
    """Convert to signed I.F fixed point, where integer_bits INCLUDES the sign bit
    (e.g. 1.31 covers [-1, 1), 9.23 covers [-256, 256))."""
    total_bits = integer_bits + fractional_bits
    fixed_point_value = round(number * (1 << fractional_bits))

    # Check range after rounding so values just below the max can't overflow
    min_value = -(1 << (total_bits - 1))
    max_value = (1 << (total_bits - 1)) - 1
    if not (min_value <= fixed_point_value <= max_value):
        lo = min_value / (1 << fractional_bits)
        hi = max_value / (1 << fractional_bits)
        print(f"{integer_bits}.{fractional_bits}: out of range [{lo}, {hi}]")
        return

    # Two's complement representation for the hex output
    raw = fixed_point_value & ((1 << total_bits) - 1)
    hex_digits = (total_bits + 3) // 4
    print(f"{integer_bits}.{fractional_bits}: {fixed_point_value} (Hex: 0x{raw:0{hex_digits}X})")


if __name__ == "__main__":
    main()
