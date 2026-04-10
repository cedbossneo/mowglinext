#!/usr/bin/env python3
"""Generate rosserial-compatible C++ headers from ROS2 .msg files.

Reads .msg definitions from ros2/src/mowgli_interfaces/msg/ and writes
C++ headers to firmware/stm32/ros_usbnode/src/ros/ros_lib/mower_msgs/.

Usage:
    python3 firmware/scripts/sync_ros_lib.py          # from repo root
    python3 firmware/scripts/sync_ros_lib.py --check  # diff-only, no writes
"""

import argparse
import hashlib
import os
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
MSG_DIR = REPO_ROOT / "ros2" / "src" / "mowgli_interfaces" / "msg"
OUT_DIR = REPO_ROOT / "firmware" / "stm32" / "ros_usbnode" / "src" / "ros" / "ros_lib" / "mower_msgs"

# ROS primitive type -> C++ type, byte size, is_signed
PRIMITIVES = {
    "bool":    ("bool",     1, False),
    "uint8":   ("uint8_t",  1, False),
    "int8":    ("int8_t",   1, True),
    "uint16":  ("uint16_t", 2, False),
    "int16":   ("int16_t",  2, True),
    "uint32":  ("uint32_t", 4, False),
    "int32":   ("int32_t",  4, True),
    "uint64":  ("uint64_t", 8, False),
    "int64":   ("int64_t",  8, True),
    "float32": ("float",    4, False),
    "float64": ("double",   8, False),
    "string":  ("const char*", 0, False),
}

# Map complex ROS types to their include paths
COMPLEX_INCLUDES = {
    "std_msgs/Header":                  "std_msgs/Header.h",
    "geometry_msgs/Pose":               "geometry_msgs/Pose.h",
    "geometry_msgs/PoseWithCovariance": "geometry_msgs/PoseWithCovariance.h",
    "geometry_msgs/PoseStamped":        "geometry_msgs/PoseStamped.h",
    "geometry_msgs/Point":              "geometry_msgs/Point.h",
    "geometry_msgs/Vector3":            "geometry_msgs/Vector3.h",
    "geometry_msgs/Polygon":            "geometry_msgs/Polygon.h",
    "builtin_interfaces/Time":          "ros/time.h",
    "nav_msgs/Path":                    "nav_msgs/Path.h",
}


def parse_msg(msg_path: Path):
    """Parse a .msg file into constants and fields."""
    constants = []
    fields = []
    for line in msg_path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        # Constant: "type NAME=value"
        m = re.match(r"^(\S+)\s+([A-Z_][A-Z0-9_]*)=(.+)$", line)
        if m:
            constants.append((m.group(1), m.group(2), m.group(3).strip()))
            continue
        # Field: "type name" or "type[] name"
        m = re.match(r"^(\S+?)(\[\])?\s+(\w+)$", line)
        if m:
            fields.append((m.group(1), m.group(3), m.group(2) is not None))
    return constants, fields


def cpp_type(ros_type: str) -> str:
    """Convert a ROS type to its C++ equivalent."""
    if ros_type in PRIMITIVES:
        return PRIMITIVES[ros_type][0]
    if ros_type == "builtin_interfaces/Time":
        return "ros::Time"
    # Complex type: "pkg/Type" -> "pkg::Type"
    if "/" in ros_type:
        pkg, name = ros_type.split("/", 1)
        return f"{pkg}::{name}"
    # Local message reference
    return f"mower_msgs::{ros_type}"


def type_size(ros_type: str) -> int:
    """Return byte size for primitive types, 0 for variable/complex."""
    if ros_type in PRIMITIVES:
        return PRIMITIVES[ros_type][1]
    return 0


def default_value(ros_type: str) -> str:
    if ros_type == "string":
        return '""'
    if ros_type in PRIMITIVES:
        return "0"
    return ""


def compute_md5(msg_path: Path) -> str:
    """Compute a simple MD5 from the message definition text."""
    text = msg_path.read_text().strip()
    return hashlib.md5(text.encode()).hexdigest()


def gen_serialize_field(ros_type: str, name: str, is_array: bool) -> list[str]:
    """Generate serialize code for a single field."""
    lines = []
    if is_array:
        lines.append(f"      *(outbuffer + offset + 0) = (this->{name}_length >> (8 * 0)) & 0xFF;")
        lines.append(f"      *(outbuffer + offset + 1) = (this->{name}_length >> (8 * 1)) & 0xFF;")
        lines.append(f"      *(outbuffer + offset + 2) = (this->{name}_length >> (8 * 2)) & 0xFF;")
        lines.append(f"      *(outbuffer + offset + 3) = (this->{name}_length >> (8 * 3)) & 0xFF;")
        lines.append(f"      offset += sizeof(this->{name}_length);")
        lines.append(f"      for( uint32_t i = 0; i < {name}_length; i++){{")
        inner = gen_serialize_field(ros_type, f"{name}[i]", False)
        lines.extend("  " + l for l in inner)
        lines.append(f"      }}")
        return lines

    if ros_type == "string":
        lines.append(f"      uint32_t length_{name} = strlen(this->{name});")
        lines.append(f"      varToArr(outbuffer + offset, length_{name});")
        lines.append(f"      offset += 4;")
        lines.append(f"      memcpy(outbuffer + offset, this->{name}, length_{name});")
        lines.append(f"      offset += length_{name};")
    elif ros_type == "bool":
        lines.append(f"      union {{")
        lines.append(f"        bool real;")
        lines.append(f"        uint8_t base;")
        lines.append(f"      }} u_{name};")
        lines.append(f"      u_{name}.real = this->{name};")
        lines.append(f"      *(outbuffer + offset + 0) = (u_{name}.base >> (8 * 0)) & 0xFF;")
        lines.append(f"      offset += sizeof(this->{name});")
    elif ros_type in PRIMITIVES:
        ctype = PRIMITIVES[ros_type][0]
        size = PRIMITIVES[ros_type][1]
        signed = PRIMITIVES[ros_type][2]
        if size == 1:
            lines.append(f"      *(outbuffer + offset + 0) = (this->{name} >> (8 * 0)) & 0xFF;")
            lines.append(f"      offset += sizeof(this->{name});")
        else:
            utype = f"uint{size*8}_t"
            if signed:
                lines.append(f"      union {{")
                lines.append(f"        {ctype} real;")
                lines.append(f"        {utype} base;")
                lines.append(f"      }} u_{name};")
                lines.append(f"      u_{name}.real = this->{name};")
                for i in range(size):
                    lines.append(f"      *(outbuffer + offset + {i}) = (u_{name}.base >> (8 * {i})) & 0xFF;")
            else:
                lines.append(f"      union {{")
                lines.append(f"        {ctype} real;")
                lines.append(f"        {utype} base;")
                lines.append(f"      }} u_{name};")
                lines.append(f"      u_{name}.real = this->{name};")
                for i in range(size):
                    lines.append(f"      *(outbuffer + offset + {i}) = (u_{name}.base >> (8 * {i})) & 0xFF;")
            lines.append(f"      offset += sizeof(this->{name});")
    else:
        # Complex type - delegate to nested serialize
        lines.append(f"      offset += this->{name}.serialize(outbuffer + offset);")
    return lines


def gen_deserialize_field(ros_type: str, name: str, is_array: bool) -> list[str]:
    """Generate deserialize code for a single field."""
    lines = []
    if is_array:
        lines.append(f"      uint32_t {name}_lengthT = ((uint32_t) (*(inbuffer + offset)));")
        lines.append(f"      {name}_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);")
        lines.append(f"      {name}_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);")
        lines.append(f"      {name}_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);")
        lines.append(f"      offset += sizeof(this->{name}_length);")
        ct = cpp_type(ros_type)
        lines.append(f"      if({name}_lengthT > {name}_length)")
        lines.append(f"        this->{name} = ({ct}*)realloc(this->{name}, {name}_lengthT * sizeof({ct}));")
        lines.append(f"      {name}_length = {name}_lengthT;")
        lines.append(f"      for( uint32_t i = 0; i < {name}_length; i++){{")
        inner = gen_deserialize_field(ros_type, f"{name}[i]", False)
        lines.extend("  " + l for l in inner)
        lines.append(f"      }}")
        return lines

    if ros_type == "string":
        lines.append(f"      uint32_t length_{name};")
        lines.append(f"      arrToVar(length_{name}, (inbuffer + offset));")
        lines.append(f"      offset += 4;")
        lines.append(f"      for(unsigned int k= offset; k< offset+length_{name}; ++k){{")
        lines.append(f"          inbuffer[k-1]=inbuffer[k];")
        lines.append(f"      }}")
        lines.append(f"      inbuffer[offset+length_{name}-1]=0;")
        lines.append(f"      this->{name} = (char *)(inbuffer + offset-1);")
        lines.append(f"      offset += length_{name};")
    elif ros_type == "bool":
        lines.append(f"      union {{")
        lines.append(f"        bool real;")
        lines.append(f"        uint8_t base;")
        lines.append(f"      }} u_{name};")
        lines.append(f"      u_{name}.base = 0;")
        lines.append(f"      u_{name}.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);")
        lines.append(f"      this->{name} = u_{name}.real;")
        lines.append(f"      offset += sizeof(this->{name});")
    elif ros_type in PRIMITIVES:
        ctype = PRIMITIVES[ros_type][0]
        size = PRIMITIVES[ros_type][1]
        signed = PRIMITIVES[ros_type][2]
        if size == 1:
            cast = f"({ctype})" if ros_type != "uint8" else "(uint8_t)"
            lines.append(f"      this->{name} =  ({cast} (*(inbuffer + offset)));")
            lines.append(f"      offset += sizeof(this->{name});")
        else:
            utype = f"uint{size*8}_t"
            lines.append(f"      union {{")
            lines.append(f"        {ctype} real;")
            lines.append(f"        {utype} base;")
            lines.append(f"      }} u_{name};")
            lines.append(f"      u_{name}.base = 0;")
            for i in range(size):
                lines.append(f"      u_{name}.base |= (({utype}) (*(inbuffer + offset + {i}))) << (8 * {i});")
            lines.append(f"      this->{name} = u_{name}.real;")
            lines.append(f"      offset += sizeof(this->{name});")
    else:
        lines.append(f"      offset += this->{name}.deserialize(inbuffer + offset);")
    return lines


def generate_header(msg_name: str, constants, fields, md5: str) -> str:
    """Generate a complete rosserial-compatible C++ header."""
    guard = f"_ROS_mower_msgs_{msg_name}_h"
    lines = [
        f"#ifndef {guard}",
        f"#define {guard}",
        "",
        "// Auto-generated by firmware/scripts/sync_ros_lib.py",
        "// Do not edit manually — re-run the script after changing .msg files.",
        "",
        "#include <stdint.h>",
        "#include <string.h>",
        "#include <stdlib.h>",
        '#include "ros/msg.h"',
    ]

    # Collect includes for complex types
    includes = set()
    for ros_type, _, is_array in fields:
        if ros_type not in PRIMITIVES and ros_type != "string":
            full_type = ros_type if "/" in ros_type else f"mower_msgs/{ros_type}"
            if full_type in COMPLEX_INCLUDES:
                includes.add(COMPLEX_INCLUDES[full_type])
            elif "/" in ros_type:
                includes.add(f"{ros_type}.h")
            else:
                includes.add(f"mower_msgs/{ros_type}.h")

    for inc in sorted(includes):
        lines.append(f'#include "{inc}"')

    lines.extend([
        "",
        "namespace mower_msgs",
        "{",
        "",
        f"  class {msg_name} : public ros::Msg",
        "  {",
        "    public:",
    ])

    # Fields with typedefs
    for ros_type, name, is_array in fields:
        ct = cpp_type(ros_type)
        if is_array:
            lines.append(f"      uint32_t {name}_length;")
            lines.append(f"      typedef {ct} _{name}_type;")
            lines.append(f"      _{name}_type st_{name};")
            lines.append(f"      _{name}_type * {name};")
        else:
            lines.append(f"      typedef {ct} _{name}_type;")
            lines.append(f"      _{name}_type {name};")

    # Constants as enums
    for const_type, const_name, const_val in constants:
        lines.append(f"      enum {{ {const_name} = {const_val} }};")

    lines.append("")

    # Constructor
    ctor_inits = []
    for ros_type, name, is_array in fields:
        if is_array:
            ctor_inits.append(f"      {name}_length(0), st_{name}(), {name}(nullptr)")
        else:
            dv = default_value(ros_type)
            if dv:
                ctor_inits.append(f"      {name}({dv})")
            else:
                ctor_inits.append(f"      {name}()")
    lines.append(f"    {msg_name}():")
    lines.append(",\n".join(ctor_inits))
    lines.append("    {")
    lines.append("    }")

    # serialize
    lines.append("")
    lines.append("    virtual int serialize(unsigned char *outbuffer) const override")
    lines.append("    {")
    lines.append("      int offset = 0;")
    for ros_type, name, is_array in fields:
        lines.extend(gen_serialize_field(ros_type, name, is_array))
    lines.append("      return offset;")
    lines.append("    }")

    # deserialize
    lines.append("")
    lines.append("    virtual int deserialize(unsigned char *inbuffer) override")
    lines.append("    {")
    lines.append("      int offset = 0;")
    for ros_type, name, is_array in fields:
        lines.extend(gen_deserialize_field(ros_type, name, is_array))
    lines.append("     return offset;")
    lines.append("    }")

    # getType / getMD5
    lines.append("")
    lines.append(f'    virtual const char * getType() override {{ return "mower_msgs/{msg_name}"; }};')
    lines.append(f'    virtual const char * getMD5() override {{ return "{md5}"; }};')

    lines.extend([
        "",
        "  };",
        "",
        "}",
        "#endif",
        "",
    ])

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true",
                        help="Check for drift without writing files")
    args = parser.parse_args()

    if not MSG_DIR.is_dir():
        print(f"Error: msg directory not found: {MSG_DIR}", file=sys.stderr)
        sys.exit(1)

    msg_files = sorted(MSG_DIR.glob("*.msg"))
    if not msg_files:
        print(f"No .msg files found in {MSG_DIR}", file=sys.stderr)
        sys.exit(1)

    OUT_DIR.mkdir(parents=True, exist_ok=True)

    changed = []
    up_to_date = []

    for msg_path in msg_files:
        msg_name = msg_path.stem
        constants, fields = parse_msg(msg_path)
        md5 = compute_md5(msg_path)
        header = generate_header(msg_name, constants, fields, md5)

        out_path = OUT_DIR / f"{msg_name}.h"
        if out_path.exists() and out_path.read_text() == header:
            up_to_date.append(msg_name)
            continue

        changed.append(msg_name)
        if args.check:
            if out_path.exists():
                print(f"  DRIFT  {msg_name}.h")
            else:
                print(f"  NEW    {msg_name}.h")
        else:
            out_path.write_text(header)
            print(f"  wrote  {msg_name}.h")

    if up_to_date:
        print(f"\n{len(up_to_date)} header(s) already up to date.")

    if args.check and changed:
        print(f"\n{len(changed)} header(s) out of sync. Run without --check to update.")
        sys.exit(1)
    elif not changed:
        print("\nAll headers up to date.")


if __name__ == "__main__":
    main()
