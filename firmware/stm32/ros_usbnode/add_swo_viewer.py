from swo_parser import swo_parser_main
import subprocess
from os import path, __file__
import time
import sys
Import("env")

def swo_viewer_task(*args, **kwargs):
    print("Entrypoint")
    board = env.BoardConfig()
    platform = env.PioPlatform()
    debug = board.manifest.get("debug", {})
    openocd_path = path.join(platform.get_package_dir("tool-openocd"), "bin", "openocd")
    upload_protocol = env.subst("$UPLOAD_PROTOCOL")
    #connect_to_openocd_tcl()
    server_args = [
        "-s",
        path.join(platform.get_package_dir("tool-openocd"), "scripts")
    ]
    if debug.get("openocd_board"):
        server_args.extend([
            "-f", "board/%s.cfg" % debug.get("openocd_board")
        ])
    else:
        assert debug.get("openocd_target"), (
            "Missed target configuration for %s" % board.id)
        server_args.extend([
            "-f", "interface/%s.cfg" % upload_protocol,
            "-c", "transport select %s" % (
                "hla_swd" if upload_protocol == "stlink" else "swd"),
            "-f", "target/%s.cfg" % debug.get("openocd_target")
        ])
    # per https://openocd.org/doc-release/pdf/openocd.pdf
    # TRACECLKIN_freq: this should be specified to match target's current TRACECLKIN frequency (usually the same as HCLK).
    # trace_freq: trace port frequency. Can be omitted in internal mode to let the adapter driver select the maximum supported rate automatically.
    swo_trace_clkin_freq = str(env.GetProjectOption("swo_trace_clkin_freq", env.subst("$BOARD_F_CPU")[:-1]))
    swo_trace_freq = str(env.GetProjectOption("swo_trace_freq", "115200"))
    server_args.extend([
        "-c", "init; tpiu config internal - uart false %s %s; itm ports on" % (swo_trace_clkin_freq, swo_trace_freq),
        #"-c", "reset run" 
        # SWO parser.py was extended to make target run
    ])
    server_args.insert(0, openocd_path)
    print("Starting OpenOCD with SWO Trace clock-in frequency %s, SWO trace frequency %s. Invocation:" % (swo_trace_clkin_freq, swo_trace_freq))
    print(server_args)
    # start client process in parallel
    subprocess.Popen([env.subst("$PYTHONEXE"), path.join(env.subst("$PROJECT_DIR"), "swo_parser.py")])
    # start openocd process parallel but wait in-line
    openocd_process = subprocess.Popen(server_args)
    openocd_process.communicate()
    print("Exited from TCL client")
    sys.exc_clear()

env.AddCustomTarget(
    "swo_viewer",
    None,
    swo_viewer_task,
    title="SWO Viewer",
    description="Starts viewing the SWO output",
    always_build=True
)
