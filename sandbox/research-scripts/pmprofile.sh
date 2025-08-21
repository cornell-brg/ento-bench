#!/bin/bash

# Defaults for optional arguments
DEFAULT_SLEEPTIME=0.1  # Default sleep time in seconds
DEFAULT_PORT=3333      # Default GDB port
DEFAULT_SAMPLES=10     # Default number of samples

# Usage function
usage() {
    echo "Usage: $0 <elf_file> [sleeptime_seconds] [port] [samples]"
    echo "  elf_file           Path to the ELF file (required)"
    echo "  sleeptime_seconds  Sleep time between samples (default: $DEFAULT_SLEEPTIME)"
    echo "  port               GDB remote port (default: $DEFAULT_PORT)"
    echo "  samples            Number of samples to collect (default: $DEFAULT_SAMPLES)"
    exit 1
}

# Check if ELF file is provided
if [ "$#" -lt 1 ]; then
    echo "Error: ELF file is required."
    usage
fi

# Parse arguments
elf_file=$1
sleeptime=${2:-$DEFAULT_SLEEPTIME}
port=${3:-$DEFAULT_PORT}
nsamples=${4:-$DEFAULT_SAMPLES}

# Validate ELF file
if [ ! -f "$elf_file" ]; then
    echo "Error: ELF file '$elf_file' not found!"
    usage
fi

# Validate sleeptime
if ! [[ $sleeptime =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
    echo "Error: Invalid sleep time '$sleeptime'. Must be a positive number."
    usage
fi

# Validate port
if ! [[ $port =~ ^[0-9]+$ ]]; then
    echo "Error: Invalid port '$port'. Must be a positive integer."
    usage
fi

# Validate samples
if ! [[ $nsamples =~ ^[0-9]+$ ]]; then
    echo "Error: Invalid number of samples '$nsamples'. Must be a positive integer."
    usage
fi

# Status message
echo "Starting profiling..."
echo "ELF file: $elf_file"
echo "Number of samples: $nsamples"
echo "Sleep time between samples: ${sleeptime}s"
echo "Connecting to target via gdb on port $port..."

# Sampling loop
for x in $(seq 1 $nsamples); do
    echo "Sampling $x of $nsamples..."
    arm-none-eabi-gdb \
        -ex "file $elf_file" \
        -ex "target remote :$port" \
        -ex "set pagination 0" \
        -ex "monitor halt" \
        -ex "bt" \
        -ex "monitor resume" \
        --batch
    sleep $sleeptime
done | \
awk '
BEGIN { s = ""; } 
/^#/ { if (s != "") { s = s "," $4 } else { s = $4 } } 
END { print s }' | \
sort | uniq -c | sort -r -n -k 1,1

# Final status
echo "Profiling complete. Processed $nsamples samples."
