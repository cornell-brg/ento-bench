import gdb
import struct

class SolveTraceBreakpoint(gdb.Breakpoint):
    def __init__(self, function_name, output_base):
        # Set a breakpoint on the given function.
        super(SolveTraceBreakpoint, self).__init__(function_name, gdb.BP_BREAKPOINT, internal=False)
        self.function_name = function_name
        self.output_base = output_base
        self.call_count = 0

    def stop(self):
        # This method is called when the breakpoint is hit.
        self.call_count += 1
        filename = f"{self.output_base}_{self.call_count}.bin"
        gdb.write(f"→ Tracing call #{self.call_count} to {self.function_name} → {filename}\n")
        with open(filename, "wb") as f:
            step = 0
            # Save the initial frame to know when the function returns.
            initial_frame = gdb.newest_frame()
            while True:
                try:
                    pc = int(gdb.parse_and_eval("$pc"))
                except gdb.error:
                    break
                try:
                    cycles = int(gdb.parse_and_eval("*(unsigned int*)0xE0001004"))
                except gdb.error:
                    cycles = 0

                try:
                    sp = int(gdb.parse_and_eval("$sp")) & 0xFFFFFFFF
                except gdb.error:
                    sp = 0
                try:
                    lr = int(gdb.parse_and_eval("$lr")) & 0xFFFFFFFF
                except gdb.error:
                    lr = 0
                try:
                    psr = int(gdb.parse_and_eval("$xpsr")) & 0xFFFFFFFF
                except gdb.error:
                    psr = 0

                # Pack the record (24 bytes: step, pc, cycles, sp, lr, psr)
                record = struct.pack("<IIIIII", step, pc, cycles, sp, lr, psr)
                f.write(record)
                step += 1

                # Step one instruction.
                gdb.execute("stepi", to_string=True)

                # Check if we've returned from the ROI.
                current_frame = gdb.newest_frame()
                if current_frame is None or current_frame.older() is not None:
                    break

        # Return False so that execution continues after tracing.
        return False

class EntoROI(gdb.Command):
    """
    Set up instruction-level ROI tracing on a function call.
    Usage: ento_trace_roi <function_name> <output_base>
    
    This command sets a breakpoint on the specified function. Each time the function is called,
    it traces every instruction until it returns (determined by tracking the stack frame) and
    writes the trace to a binary file. Files are named as <output_base>_1.bin, <output_base>_2.bin, etc.
    """
    def __init__(self):
        super(EntoROI, self).__init__("ento_trace_roi", gdb.COMMAND_USER)

    def invoke(self, args, from_tty):
        tokens = gdb.string_to_argv(args)
        if len(tokens) != 2:
            gdb.write("Usage: ento_trace_roi <function_name> <output_base>\n")
            return

        function_name = tokens[0]
        output_base = tokens[1]
        SolveTraceBreakpoint(function_name, output_base)
        gdb.write(f"[ento-perf] ROI tracer armed on {function_name} with output base '{output_base}'\n")

# Register the command with GDB.
EntoROI()
