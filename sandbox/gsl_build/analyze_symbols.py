import sys

def parse_symbols(symbols_file):
    symbols = []
    with open(symbols_file, 'r') as file:
        for line in file:
            parts = line.split()
            if len(parts) < 4:
                continue
            address = parts[0]
            size = int(parts[1], 16)
            type = parts[2]
            name = parts[3]
            symbols.append((address, size, type, name))
    return symbols

def main():
    if len(sys.argv) != 2:
        print("Usage: python analyze_symbols.py <symbols_file>")
        sys.exit(1)

    symbols_file = sys.argv[1]
    symbols = parse_symbols(symbols_file)

    total_gsl_size = 0
    total_cblas_size = 0
    gsl_symbols = []
    cblas_symbols = []

    for address, size, type, name in symbols:
        if 'gsl' in name and type in ['T', 't']:  # Only consider text symbols
            total_gsl_size += size
            gsl_symbols.append((name, size))
        if 'cblas' in name and type in ['T', 't']:  # Only consider text symbols
            total_cblas_size += size
            cblas_symbols.append((name, size))
    print(symbols)

    print(f"Total size of 'gsl' symbols in .text section: {total_gsl_size} bytes")
    print(f"Total size of 'cblas' symbols in .text section: {total_cblas_size} bytes")
    
    # Save gsl symbols to a file
    with open("gsl_symbols_used.txt", 'w') as gsl_file:
        for name, size in gsl_symbols:
            gsl_file.write(f"{name} {size}\n")
    
    # Save cblas symbols to a file
    with open("cblas_symbols_used.txt", 'w') as cblas_file:
        for name, size in cblas_symbols:
            cblas_file.write(f"{name} {size}\n")

if __name__ == "__main__":
    main()

