def hex_to_c_array(input_file, output_file, array_name):
    with open(input_file, 'r') as f:
        hex_data = f.read().strip()

    # Remove the colon marks from the hex data
    hex_data = hex_data.replace(':', '')

    # Split the hex data into 2-character chunks
    hex_bytes = [hex_data[i:i+2] for i in range(0, len(hex_data), 2)]

    # Open the output file for writing
    with open(output_file, 'w') as f:
        # Write the array declaration
        f.write(f"const unsigned char {array_name}[] = {{\n")

        # Write each byte in hex format
        for i, byte in enumerate(hex_bytes):
            if i % 16 == 0:
                f.write("    ")
            f.write(f"0x{byte.upper()}")
            if i < len(hex_bytes) - 1:
                f.write(", ")
            if (i + 1) % 16 == 0 and i < len(hex_bytes) - 1:
                f.write("\n")

        # Write the array size declaration
        f.write(f"\n}};\n\n")
        f.write(f"const unsigned int {array_name}_size = sizeof({array_name});\n")

# Example usage:
input_file = "zephyr.hex"
output_file = "firmware_data.h"
array_name = "firmware_data"

hex_to_c_array(input_file, output_file, array_name)
