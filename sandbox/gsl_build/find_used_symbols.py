import re
import argparse

def extract_gsl_symbols(file_path):
    """Extract symbols containing 'gsl' from the objdump output."""
    symbols = set()
    with open(file_path, 'r') as file:
        for line in file:
            if 'gsl' in line:
                match = re.search(r'\b(gsl[a-zA-Z_][a-zA-Z0-9_]*)\b', line)
                if match:
                    symbol = match.group(1).strip()
                    symbols.add(symbol)
    return symbols

def main():
    parser = argparse.ArgumentParser(description='Count GSL symbols in an executable and a static library.')
    parser.add_argument('main_symbols', help='Path to the main executable objdump symbols file')
    parser.add_argument('lib_symbols', help='Path to the static library objdump symbols file')
    args = parser.parse_args()

    # Extract GSL symbols from the main executable objdump output
    main_symbols = extract_gsl_symbols(args.main_symbols)

    # Extract GSL symbols from the static library objdump output
    lib_symbols = extract_gsl_symbols(args.lib_symbols)

    # Count how many library symbols are used by the main executable
    used_symbols = main_symbols.intersection(lib_symbols)

    # Print the counts
    print(f"Total GSL symbols in libgsl: {len(lib_symbols)}")
    print(f"GSL symbols used by main: {len(used_symbols)}")

if __name__ == '__main__':
    main()

