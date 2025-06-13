#!/usr/bin/env python3
import argparse
import os
import sys
from pathlib import Path

def safe_rename(path: str, dry_run: bool, auto_confirm: bool) -> bool:
    """
    Safely renames files in a directory to match the directory's name.

    Args:
        path (str): The path to the target directory.
        dry_run (bool): If True, only print the planned changes.
        auto_confirm (bool): If True, do not prompt for confirmation.

    Returns:
        bool: The updated state of auto_confirm.
    """
    target_dir = Path(path).resolve()
    if not target_dir.is_dir():
        # This can happen if a subdirectory was renamed in recursive mode
        if not dry_run: return auto_confirm
        print(f"Warning: Path '{target_dir}' is not a valid directory. Skipping.")
        return auto_confirm

    new_base_name = target_dir.name

    print(f"Directory:     {target_dir}")
    print(f"New base name: '{new_base_name}'")

    # Use sorted() for a predictable execution order
    files_to_process = [item for item in sorted(target_dir.iterdir()) if item.is_file() and not item.name.startswith('.')]

    if not files_to_process:
        print("No files to process in this directory.")
        return auto_confirm

    for item in files_to_process:
        # Naming logic
        file_extensions = "".join(item.suffixes)
        stem = item.name.removesuffix(file_extensions)

        if stem == new_base_name:
            print(f"Skipping (already correct): '{item.name}'")
            continue

        new_name_with_ext = f"{new_base_name}{file_extensions}"
        new_path = target_dir / new_name_with_ext

        # Conflict resolution
        counter = 1
        while new_path.exists() and not new_path.samefile(item):
            new_name_with_ext = f"{new_base_name}-{counter}{file_extensions}"
            new_path = target_dir / new_name_with_ext
            counter += 1

        print(f"Plan: mv '{item.name}'  ->  '{new_path.name}'")

        if not dry_run:
            do_rename = auto_confirm
            if not auto_confirm:
                prompt = input("  └── Rename this file? [y(es), n(o), a(ll), q(uit)]: ").lower()
                if prompt in ('y', 'yes'):
                    do_rename = True
                elif prompt in ('a', 'all'):
                    do_rename = True
                    auto_confirm = True
                elif prompt in ('q', 'quit'):
                    print("Quitting.")
                    sys.exit(0)
                else:
                    print("      └── Skipped.")
                    continue

            if do_rename:
                try:
                    item.rename(new_path)
                    print("      └── Renamed successfully.")
                except OSError as e:
                    print(f"      └── ERROR: Could not rename. {e}")
    return auto_confirm

def main():
    parser = argparse.ArgumentParser(
        description="A safer script to rename files to match their parent directory.",
        epilog="""Examples:
  Dry Run (single dir):  python %(prog)s /path/to/my-data
  Execute (single dir):  python %(prog)s /path/to/my-data --execute
  Recursive Dry Run:     python %(prog)s /path/to/top-level -r
  Recursive Execute:     python %(prog)s /path/to/top-level -r --execute""",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument("path", help="Path to the directory to process.")
    parser.add_argument(
        "-r", "--recursive", action="store_true",
        help="Recursively find and process all subdirectories."
    )
    parser.add_argument(
        "--execute", action="store_true",
        help="Perform the renaming. Default is a dry run.",
    )
    args = parser.parse_args()

    if args.recursive:
        print("Mode:          Recursive Execute" if args.execute else "Mode:          Recursive Dry Run")
        dirs_to_process = [root for root, _, _ in os.walk(args.path)]
    else:
        print("Mode:          Execute" if args.execute else "Mode:          Dry Run")
        dirs_to_process = [args.path]

    print("-" * 40)
    auto_confirm_all = False
    for i, dir_path in enumerate(sorted(dirs_to_process)):
        if i > 0: print("\n" + "="*40)
        auto_confirm_all = safe_rename(
            path=dir_path,
            dry_run=not args.execute,
            auto_confirm=auto_confirm_all
        )

if __name__ == "__main__":
    main()

