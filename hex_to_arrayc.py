def hex_to_c_array(input_file, output_file, array_name):
    with open(input_file, 'rb') as f:
        hex_data = f.read()

    # Open the output file for writing
    with open(output_file, 'w') as f:
        # Write the array declaration
        f.write(f"const unsigned char {array_name}[] = {{\n")

        # Write each byte in hex format
        for i, byte in enumerate(hex_data):
            if i % 16 == 0:
                f.write("    ")
            f.write(f"0x{byte:02X}")
            if i < len(hex_data) - 1:
                f.write(", ")
            if (i + 1) % 16 == 0 and i < len(hex_data) - 1:
                f.write("\n")

        # Write the array size declaration
        f.write(f"\n}};\n\n")
        f.write(f"const unsigned int {array_name}_size = sizeof({array_name});\n")

# Example usage:
input_file = "zephyr.hex"
output_file = "firmware_data.h"
array_name = "firmware_data"

hex_to_c_array(input_file, output_file, array_name)
