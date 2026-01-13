#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
UI Helper Module
Handles user interaction and mode selection.
Keeps main_node.py clean and focused on mission logic.
"""

import os
import glob


def list_excel_files(package_dir):
    """
    List available Excel files in data/excel directory.

    Args:
        package_dir: Package root directory

    Returns:
        List of Excel file paths
    """
    excel_dir = os.path.join(package_dir, 'data', 'excel')

    if not os.path.exists(excel_dir):
        return []

    # Find all Excel files
    excel_files = []
    for ext in ['*.xlsx', '*.xls']:
        excel_files.extend(glob.glob(os.path.join(excel_dir, ext)))

    return sorted(excel_files)


def select_excel_file(package_dir):
    """
    Interactive Excel file selection.

    Args:
        package_dir: Package root directory

    Returns:
        Selected Excel file path or None
    """
    excel_files = list_excel_files(package_dir)

    if not excel_files:
        print("\n[ERROR] No Excel files found in data/excel/ directory!")
        print("Please place your Excel files in: data/excel/\n")
        return None

    print("\nAvailable Excel files:")
    print("-" * 50)
    for i, filepath in enumerate(excel_files, 1):
        filename = os.path.basename(filepath)
        print(f"{i}: {filename}")
    print("-" * 50)

    while True:
        try:
            choice = input(f"\nSelect file (1-{len(excel_files)}): ").strip()
            idx = int(choice) - 1
            if 0 <= idx < len(excel_files):
                return excel_files[idx]
            print(f"Please enter a number between 1 and {len(excel_files)}")
        except (ValueError, KeyboardInterrupt, EOFError):
            return None


def select_mode():
    """
    Interactive mode selection.

    Returns:
        (mode, target_tag/excel_path)
    """
    print("=" * 50)
    print("AprilTag Navigation System")
    print("=" * 50)
    print("\n1: Task 1 (Zone B + C)")
    print("2: Task 2 (Zone D + E)")
    print("3: Direct Navigation (Go to specific tag)")
    print("4: Scan Mode (from Excel)\n")

    while True:
        try:
            choice = input("Select mode (1/2/3/4): ").strip()
            if choice == '1':
                return 1, None
            elif choice == '2':
                return 2, None
            elif choice == '3':
                target_tag = input("Enter target tag ID: ").strip()
                if target_tag.isdigit():
                    return 3, int(target_tag)
                print("Please enter a valid tag ID (number)!")
            elif choice == '4':
                # Get package directory from caller
                package_dir = os.path.join(os.path.dirname(__file__), '..', '..')
                excel_path = select_excel_file(package_dir)
                if excel_path:
                    return 4, excel_path
                print("Excel file selection cancelled!")
        except (KeyboardInterrupt, EOFError):
            return None, None


def print_banner(title):
    """
    Print a formatted banner.

    Args:
        title: Banner title
    """
    print("=" * 60)
    print(title)
    print("=" * 60)
