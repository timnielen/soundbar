import struct, sys
raw_path, name, out_path = sys.argv[1], sys.argv[2], sys.argv[3]
raw = open(raw_path, "rb").read()
s = struct.unpack("<%dh" % (len(raw) // 2), raw)
with open(out_path, "w", encoding="ascii", newline="\n") as f:
    f.write('#include <stdint.h>\n\n')
    f.write(f"const int16_t {name}[{len(s)}] = {{\n")
    for i in range(0, len(s), 12):
        f.write("    " + ", ".join(map(str, s[i:i+12])) + ",\n")
    f.write("};\n")