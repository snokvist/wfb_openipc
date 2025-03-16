#!/usr/bin/env python3
import socket
import time
import logging
import sys
import os
import argparse
import base64
import hashlib
import tarfile
import io

# Try to import yaml for parsing INFO output; if unavailable, use fallback.
try:
    import yaml
except ImportError:
    yaml = None

# -------------------- Utility Functions --------------------

def compute_sha1(file_path):
    """Compute the SHA1 hash of the given file."""
    hash_obj = hashlib.sha1()
    with open(file_path, 'rb') as f:
        while chunk := f.read(8192):
            hash_obj.update(chunk)
    return hash_obj.hexdigest()

def compute_checksums(directory, checksum_file_path):
    """
    Compute SHA1 checksums for all files in a directory except checksum.txt.
    Returns a list of strings of the format: "<sha1>  <relative_path>"
    """
    checksum_lines = []
    for root, _, files in os.walk(directory):
        for file in files:
            file_path = os.path.join(root, file)
            # Exclude the checksum file itself (compare absolute paths)
            if os.path.abspath(file_path) == os.path.abspath(checksum_file_path):
                continue
            rel_path = os.path.relpath(file_path, start=directory)
            sha1_hash = compute_sha1(file_path)
            checksum_lines.append(f"{sha1_hash}  {rel_path}")
    return checksum_lines

def create_tar_gz_archive(source_dir, arcname):
    """
    Create a tar.gz archive of the source directory.
    Before archiving, compute the SHA1 checksums of all files (excluding checksum.txt)
    and write them into checksum.txt in the root of the folder.
    """
    bio = io.BytesIO()
    checksum_file_path = os.path.join(source_dir, "checksum.txt")
    # Compute checksums excluding checksum.txt itself.
    checksum_lines = compute_checksums(source_dir, checksum_file_path)
    
    # Write checksum.txt file
    with open(checksum_file_path, 'w') as f:
        for line in checksum_lines:
            f.write(line + "\n")
    
    # Create tar.gz archive including checksum.txt and all other files
    with tarfile.open(fileobj=bio, mode='w:gz') as tar:
        tar.add(source_dir, arcname=arcname)
    
    bio.seek(0)
    return bio.read()

def send_rate_limited(sock_file, data, bw_limit, progress=False):
    """Send data using bandwidth limiting."""
    bw_bytes_per_sec = bw_limit / 8.0
    chunk_size = 4096
    total = len(data)
    sent = 0
    start_time = time.time()
    
    while sent < total:
        end = min(sent + chunk_size, total)
        chunk = data[sent:end]
        sock_file.write(chunk)
        sock_file.flush()
        sent += len(chunk)
        elapsed = time.time() - start_time
        expected = sent / bw_bytes_per_sec
        if expected > elapsed:
            time.sleep(expected - elapsed)
        if progress:
            percent = sent / total * 100
            bar_length = 40
            filled_length = int(round(bar_length * sent / total))
            bar = '=' * filled_length + '-' * (bar_length - filled_length)
            sys.stdout.write(f'\rProgress: [{bar}] {percent:6.2f}%')
            sys.stdout.flush()
    
    if progress:
        sys.stdout.write('\n')

def connect_to_server(host, port, max_retries, conn_timeout, op_timeout):
    """Connect to the server with retries; return socket and file-like object."""
    sock = None
    for attempt in range(1, max_retries + 1):
        try:
            logging.debug(f"Attempt {attempt}: Connecting to {host}:{port} ...")
            sock = socket.create_connection((host, port), timeout=conn_timeout)
            logging.debug("Connection established.")
            break
        except Exception as e:
            logging.debug(f"Attempt {attempt} failed: {e}")
            time.sleep(1)
    if not sock:
        logging.error("Unable to connect to the server after multiple attempts.")
        sys.exit(1)
    sock.settimeout(op_timeout)
    sock_file = sock.makefile('rwb')
    return sock, sock_file

def process_response(response_line, command, debug):
    """
    Process the response received from the server.
    The response is expected to be in the format: STATUS<TAB>DATA.
    For INFO, the DATA is base64 encoded and will be decoded.
    - If debug is enabled, the STATUS (e.g. "OK" or "ERR") is sent to stderr.
    - The DATA (the actual message) is sent to stdout.
    """
    parts = response_line.split("\t", 1)
    if len(parts) == 2:
        status, data = parts
    else:
        status = parts[0]
        data = ""
    if command.upper() == "INFO":
        try:
            data = base64.b64decode(data).decode("utf-8")
        except Exception:
            pass
    if debug:
        sys.stderr.write(status + "\n")
    sys.stdout.write(data)

def get_info(args):
    """
    Fetch the INFO command's output from the server as a YAML-formatted string.
    Returns the decoded INFO string.
    """
    sock, sock_file = connect_to_server(args.ip, args.port, args.max_retries, args.conn_timeout, args.timeout)
    try:
        sock_file.write("INFO\n".encode("utf-8"))
        sock_file.flush()
        response_line = sock_file.readline().decode("utf-8").strip()
        parts = response_line.split("\t", 1)
        if len(parts) == 2 and parts[0] == "OK":
            try:
                info_text = base64.b64decode(parts[1]).decode("utf-8")
            except Exception:
                info_text = parts[1]
        else:
            info_text = response_line
    finally:
        sock_file.close()
        sock.close()
    return info_text

# -------------------- Operation Functions --------------------

def bind_operation(folder_path, args):
    """
    Perform the BIND operation.
    If folder_path is a file ending with .tar.gz, send it directly.
    Otherwise, assume it's a folder, create a tar.gz archive with checksum.txt, and send it.
    """
    host = args.ip
    port = args.port
    sock, sock_file = connect_to_server(host, port, args.max_retries, args.conn_timeout, args.timeout)
    try:
        if os.path.isfile(folder_path) and folder_path.lower().endswith('.tar.gz'):
            with open(folder_path, "rb") as f:
                file_data = f.read()
            encoded_archive = base64.b64encode(file_data).decode('utf-8')
        else:
            archive_name = os.path.basename(os.path.normpath(folder_path))
            encoded_archive = base64.b64encode(create_tar_gz_archive(folder_path, archive_name)).decode('utf-8')
        bind_message = f"BIND\t{encoded_archive}\n".encode('utf-8')
        send_rate_limited(sock_file, bind_message, args.bw_limit, progress=True)
        response_line = sock_file.readline().decode('utf-8').strip()
        process_response(response_line, "BIND", args.debug)
    finally:
        sock_file.close()
        sock.close()

def flash_operation(archive_file, args):
    """Perform the FLASH operation by sending a file as base64."""
    if not os.path.isfile(archive_file):
        logging.error(f"FLASH failed: Archive file '{archive_file}' does not exist.")
        sys.exit(1)
    host = args.ip
    port = args.port
    sock, sock_file = connect_to_server(host, port, args.max_retries, args.conn_timeout, args.timeout)
    try:
        logging.debug(f"Reading archive file: {archive_file}")
        with open(archive_file, "rb") as f:
            file_data = f.read()
        encoded_archive = base64.b64encode(file_data).decode("utf-8")
        flash_message = f"FLASH\t{encoded_archive}\n".encode("utf-8")
        send_rate_limited(sock_file, flash_message, args.bw_limit, progress=True)
        response_line = sock_file.readline().decode("utf-8").strip()
        process_response(response_line, "FLASH", args.debug)
    finally:
        sock_file.close()
        sock.close()

def simple_command_operation(command, args):
    """
    Perform INFO, VERSION, or UNBIND operations.
    Only the actual data message is output to stdout;
    any extra status (e.g. "OK" or "ERR") is sent to stderr when --debug is enabled.
    """
    sock, sock_file = connect_to_server(args.ip, args.port, args.max_retries, args.conn_timeout, args.timeout)
    try:
        sock_file.write(f"{command}\n".encode("utf-8"))
        sock_file.flush()
        response_line = sock_file.readline().decode("utf-8").strip()
        process_response(response_line, command, args.debug)
    finally:
        sock_file.close()
        sock.close()

def backup_operation(dest_folder, args):
    """
    Perform the BACKUP operation.
    1. Run the INFO command to get YAML configuration.
    2. Parse the YAML (or use a simple fallback) to extract:
         'vtx_id', 'vtx_name', 'build_option', and 'soc'.
       If the INFO data has a top-level "vtx_info" key, use its contents.
    3. Create a filename using these values and the current date/time.
    4. Send the BACKUP command to the server.
    5. Receive a base64 encoded backup file, decode it, and write it to the destination folder with rate limiting.
    """
    if not os.path.isdir(dest_folder):
        logging.error(f"Backup folder '{dest_folder}' does not exist or is not a directory.")
        sys.exit(1)
    
    # Get INFO data.
    info_text = get_info(args)
    if yaml is not None:
        try:
            info_data = yaml.safe_load(info_text)
        except Exception as e:
            logging.error("Failed to parse INFO output with YAML: " + str(e))
            info_data = {}
    else:
        # Fallback simple parsing: look for lines starting with the keys
        info_data = {}
        for line in info_text.splitlines():
            line = line.strip()
            for key in ["vtx_id", "vtx_name", "build_option", "soc"]:
                if line.startswith(key + ":"):
                    info_data[key] = line.split(":", 1)[1].strip()
    
    # If the parsed YAML has a top-level key "vtx_info", use its contents.
    if isinstance(info_data, dict) and "vtx_info" in info_data:
        info_data = info_data["vtx_info"]
    
    # Extract required keys (using defaults if missing).
    vtx_id = info_data.get("vtx_id", "unknown")
    vtx_name = info_data.get("vtx_name", "unknown")
    build_option = info_data.get("build_option", "unknown")
    soc = info_data.get("soc", "unknown")
    
    timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    backup_filename = os.path.join(dest_folder, f"{vtx_id}_{vtx_name}_{build_option}_{soc}_{timestamp}.tar.gz")
    
    host = args.ip
    port = args.port
    sock, sock_file = connect_to_server(host, port, args.max_retries, args.conn_timeout, args.timeout)
    try:
        sock_file.write(b"BACKUP\n")
        sock_file.flush()
        response_line = sock_file.readline().decode("utf-8").strip()
        parts = response_line.split("\t", 1)
        if len(parts) < 2 or parts[0] != "OK":
            logging.error("BACKUP command failed: " + (parts[1] if len(parts) > 1 else "No message"))
            sys.exit(1)
        encoded_data = parts[1]
        try:
            decoded_data = base64.b64decode(encoded_data)
        except Exception as e:
            logging.error("Failed to decode backup data: " + str(e))
            sys.exit(1)
        
        # Write decoded backup data to file with rate limiting.
        bw_bytes_per_sec = args.bw_limit / 8.0
        chunk_size = 4096
        total = len(decoded_data)
        written = 0
        start_time = time.time()
        with open(backup_filename, "wb") as backup_file:
            while written < total:
                end = min(written + chunk_size, total)
                chunk = decoded_data[written:end]
                backup_file.write(chunk)
                backup_file.flush()
                written += len(chunk)
                elapsed = time.time() - start_time
                expected = written / bw_bytes_per_sec
                if expected > elapsed:
                    time.sleep(expected - elapsed)
        logging.info(f"Backup successfully saved to: {backup_filename}")
        if args.debug:
            sys.stderr.write("BACKUP succeeded.\n")
    finally:
        sock_file.close()
        sock.close()

# -------------------- Main --------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", nargs="?", help="Path for BIND/FLASH/BACKUP operations")
    parser.add_argument("--bind", action="store_true", help="Perform BIND operation")
    parser.add_argument("--flash", action="store_true", help="Perform FLASH operation")
    parser.add_argument("--backup", action="store_true", help="Perform BACKUP operation")
    parser.add_argument("--unbind", action="store_true", help="Perform UNBIND operation")
    parser.add_argument("--info", action="store_true", help="Perform INFO operation")
    parser.add_argument("--version", action="store_true", help="Perform VERSION operation")
    parser.add_argument("--ip", "-i", default="10.5.0.10", help="Server IP address")
    parser.add_argument("--port", "-p", type=int, default=5555, help="Server port")
    parser.add_argument("--max-retries", "-r", type=int, default=30, help="Max connection retries")
    parser.add_argument("--timeout", "-t", type=int, default=60, help="Socket timeout after connection")
    parser.add_argument("--conn-timeout", "-c", type=int, default=5, help="Timeout for connection attempt")
    parser.add_argument("--bw-limit", type=int, default=2 * 1024 * 1024, help="Bandwidth limit in bits/sec")
    parser.add_argument("--debug", action="store_true", help="Enable debug output (sends extra info to stderr)")
    
    args = parser.parse_args()
    
    # Configure logging.
    if args.debug:
        logging.basicConfig(level=logging.DEBUG, format='[%(asctime)s] %(levelname)s: %(message)s', datefmt='%H:%M:%S')
    else:
        logging.basicConfig(level=logging.INFO, format='[%(asctime)s] %(levelname)s: %(message)s', datefmt='%H:%M:%S')
    
    # For operations that require a folder (BIND, FLASH, BACKUP), ensure it is provided.
    if (args.bind or args.flash or args.backup) and not args.folder:
        parser.error("The --bind, --flash, and --backup operations require a folder (for bind/backup) or file (for flash) argument.")
    
    if args.bind:
        sys.stderr.write(f"Bind initiated with folder: {args.folder}\n")
        bind_operation(args.folder, args)
    elif args.flash:
        sys.stderr.write(f"Flash initiated with file: {args.folder}\n")
        flash_operation(args.folder, args)
    elif args.backup:
        sys.stderr.write(f"Backup initiated with destination folder: {args.folder}\n")
        backup_operation(args.folder, args)
    elif args.unbind:
        sys.stderr.write("Unbind initiated.\n")
        simple_command_operation("UNBIND", args)
    elif args.info:
        sys.stderr.write("Info initiated.\n")
        simple_command_operation("INFO", args)
    elif args.version:
        sys.stderr.write("Version initiated.\n")
        simple_command_operation("VERSION", args)
    else:
        parser.print_help()

if __name__ == "__main__":
    main()
