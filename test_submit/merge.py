#!/usr/bin/env python3
"""
merge.py

将工程内的本地 `.h`/`.hpp`/`.cpp` 文件按依赖展开并合并为一个单一输出文件。

用法示例:
  python merge.py --src-dir . --main main.cpp --out merged.cpp

设计说明:
  - 从指定的 `--main` 文件开始，递归解析 `#include "..."` 本地包含，按依赖顺序展开。
  - 保留系统包含（<...>），并去重。
  - 避免重复展开已处理的文件（处理 include guards / pragma once 的常见场景）。
  - 将所有未被主入口覆盖的源/头文件也合并进来，确保输出包含所有 `.cpp`/`.h`。
"""

import argparse
import os
import re
import sys
import shutil
import subprocess
from collections import deque
from datetime import datetime


INCLUDE_RE = re.compile(r'^\s*#\s*include\s*"([^"]+)"')
SYS_INCLUDE_RE = re.compile(r'^\s*#\s*include\s*<([^>]+)>')
PRAGMA_GCC_RE = re.compile(r'^\s*#\s*pragma\s+GCC\b')


def find_all_files(src_dir):
	files = {}
	for root, _, filenames in os.walk(src_dir):
		for fn in filenames:
			if fn.endswith(('.h', '.hpp', '.hh', '.cpp', '.cc', '.cxx', '.c')):
				path = os.path.normpath(os.path.join(root, fn))
				files[os.path.relpath(path, src_dir).replace('\\', '/')]=path
	return files


def parse_includes(path):
	local = []
	system = []
	try:
		with open(path, 'r', encoding='utf-8', errors='ignore') as f:
			for line in f:
				m = INCLUDE_RE.match(line)
				if m:
					local.append(m.group(1))
					continue
				m2 = SYS_INCLUDE_RE.match(line)
				if m2:
					system.append(m2.group(1))
	except FileNotFoundError:
		pass
	return local, system


def resolve_include(include_name, current_dir, src_dir, all_files):
	# try relative to current file
	candidate = os.path.normpath(os.path.join(current_dir, include_name))
	if os.path.exists(candidate):
		rel = os.path.relpath(candidate, src_dir).replace('\\', '/')
		if rel in all_files:
			return all_files[rel]
	# try relative to src_dir
	candidate2 = os.path.normpath(os.path.join(src_dir, include_name))
	if os.path.exists(candidate2):
		rel2 = os.path.relpath(candidate2, src_dir).replace('\\', '/')
		if rel2 in all_files:
			return all_files[rel2]
	# try matching known files by relative name
	key1 = os.path.relpath(candidate, src_dir).replace('\\', '/')
	key2 = os.path.relpath(candidate2, src_dir).replace('\\', '/')
	if key1 in all_files:
		return all_files[key1]
	if key2 in all_files:
		return all_files[key2]
	# last resort: search for filename in all_files
	for rel, full in all_files.items():
		if rel.endswith('/' + include_name) or os.path.basename(rel) == include_name:
			return full
	return None


def build_order(start_path, src_dir, all_files, verbose=False, prepend_paths=None):
	visited = set()
	order = []
	system_includes = set()
	prepend_paths = prepend_paths or []

	def dfs(path):
		norm = os.path.normpath(path)
		if norm in visited:
			return
		visited.add(norm)
		curdir = os.path.dirname(norm)
		local, systems = parse_includes(norm)
		for s in systems:
			system_includes.add(s)
		for inc in local:
			resolved = resolve_include(inc, curdir, src_dir, all_files)
			if resolved:
				dfs(resolved)
			else:
				if verbose:
					print(f"warning: include '{inc}' in {norm} not found")
		order.append(norm)

	for extra_path in prepend_paths:
		if extra_path:
			dfs(extra_path)

	dfs(start_path)

	# include remaining source files only; headers are merged only when reachable from main
	source_exts = {'.cpp', '.cc', '.cxx', '.c'}
	for rel, full in all_files.items():
		if os.path.normpath(full) not in visited:
			ext = os.path.splitext(full)[1].lower()
			if ext in source_exts:
				if verbose:
					print(f"including additional file: {rel}")
				dfs(full)
			elif verbose:
				print(f"skipping extra header: {rel}")

	return order, system_includes


def strip_local_includes(text):
	lines = []
	for line in text.splitlines():
		# remove local includes "..." and system includes <...> from individual file bodies
		if INCLUDE_RE.match(line) or SYS_INCLUDE_RE.match(line) or PRAGMA_GCC_RE.match(line):
			continue
		lines.append(line)
	return '\n'.join(lines) + '\n'


def collect_gcc_pragmas(path):
	pragmas = []
	try:
		with open(path, 'r', encoding='utf-8', errors='ignore') as f:
			for line in f:
				if PRAGMA_GCC_RE.match(line):
					pragmas.append(line.rstrip('\n'))
	except FileNotFoundError:
		pass
	return pragmas


def strip_comments(text):
	"""Remove all C/C++ comments (both // and /* */) from text, including leading whitespace and line breaks."""
	result = []
	i = 0
	n = len(text)
	while i < n:
		# single-line comment
		if text[i] == '/' and i + 1 < n and text[i + 1] == '/':
			# remove leading whitespace in result before this comment
			while result and result[-1] in (' ', '\t'):
				result.pop()
			# skip until end of line, including the newline
			while i < n and text[i] != '\n':
				i += 1
			if i < n and text[i] == '\n':
				i += 1
			continue
		# multi-line comment
		if text[i] == '/' and i + 1 < n and text[i + 1] == '*':
			# remove leading whitespace in result before this comment
			while result and result[-1] in (' ', '\t'):
				result.pop()
			i += 2
			while i < n:
				if text[i] == '*' and i + 1 < n and text[i + 1] == '/':
					i += 2
					break
				i += 1
			continue
		result.append(text[i])
		i += 1
	# remove blank lines (lines that are empty or contain only whitespace)
	lines = ''.join(result).splitlines(True)
	filtered = []
	for line in lines:
		stripped = line.strip()
		if stripped == '':
			continue
		filtered.append(line)
	return ''.join(filtered)


def remove_include_guard(text):
	# remove #pragma once anywhere
	text = re.sub(r"^\s*#\s*pragma\s+once\s*$", "", text, flags=re.MULTILINE)

	# detect classic include guard at top: #ifndef MACRO / #define MACRO ... #endif
	lines = text.splitlines()
	n = len(lines)
	i = 0
	# skip initial comments/blank lines
	while i < n and (lines[i].strip().startswith('//') or lines[i].strip().startswith('/*') or lines[i].strip() == ''):
		i += 1
	if i + 1 < n:
		m1 = re.match(r'^\s*#\s*ifndef\s+(\w+)', lines[i])
		m2 = re.match(r'^\s*#\s*define\s+(\w+)', lines[i+1])
		if m1 and m2 and m1.group(1) == m2.group(1):
			# find last #endif and remove the guard
			j = n - 1
			while j > i + 1:
				if re.match(r'^\s*#\s*endif\b', lines[j]):
					new_lines = lines[:i] + lines[i+2:j] + lines[j+1:]
					return '\n'.join(new_lines) + ('\n' if new_lines and new_lines[-1] != '' else '')
				j -= 1
	return text


def merge(src_dir, main_relpath, out_path, verbose=False, exclude_list=None, prepend_list=None):
	all_files = find_all_files(src_dir)
	main_relpath = main_relpath.replace('\\', '/')
	out_path = os.path.normpath(out_path)
	out_dir = os.path.dirname(os.path.abspath(out_path))
	if out_dir and not os.path.exists(out_dir):
		os.makedirs(out_dir, exist_ok=True)
	if main_relpath not in all_files:
		# try if user passed a path directly
		main_abs = os.path.normpath(os.path.join(src_dir, main_relpath))
		if not os.path.exists(main_abs):
			raise FileNotFoundError(f"main file {main_relpath} not found in {src_dir}")
		main_path = main_abs
	else:
		main_path = all_files[main_relpath]

	# Restrict source files to those in the same directory as the main file
	main_dir = os.path.normpath(os.path.dirname(main_path))
	filtered = {}
	# prepare exclude set (match by relative path or basename)
	exclude_set = set()
	prepend_set = []
	if exclude_list:
		for item in exclude_list:
			for part in str(item).split(','):
				p = part.strip()
				if not p:
					continue
				p_norm = p.replace('\\', '/')
				exclude_set.add(p_norm)
				exclude_set.add(os.path.basename(p_norm))
	if prepend_list:
		for item in prepend_list:
			for part in str(item).split(','):
				p = part.strip()
				if not p:
					continue
				p_norm = p.replace('\\', '/')
				prepend_set.append(p_norm)

	for rel, full in all_files.items():
		# skip excluded files
		rel_norm = rel.replace('\\', '/')
		if rel_norm in exclude_set or os.path.basename(rel_norm) in exclude_set:
			if verbose:
				print(f"excluding file: {rel_norm}")
			continue
		try:
			if os.path.commonpath([main_dir, os.path.normpath(full)]) == main_dir:
				filtered[rel] = full
		except ValueError:
			# on some platforms commonpath can raise for different drives; skip those
			continue
	if not filtered:
		raise RuntimeError(f"no source files found in same directory as main: {main_dir}")

	if verbose:
		print(f"restricting to files under: {main_dir}, {len(filtered)} files")

	prepend_paths = []
	for p in prepend_set:
		resolved = resolve_include(p, src_dir, src_dir, filtered)
		if resolved:
			prepend_paths.append(resolved)
		elif os.path.exists(os.path.normpath(os.path.join(src_dir, p))):
			prepend_paths.append(os.path.normpath(os.path.join(src_dir, p)))
		elif verbose:
			print(f"warning: required header '{p}' not found")
	order, system_includes = build_order(main_path, src_dir, filtered, verbose=verbose, prepend_paths=prepend_paths)

	# Reorder: keep dependency order (post-order from DFS), but place header files before source files
	# Further: .h files with NO local includes go first (sorted by string), then other headers, then sources
	no_inc_headers = []
	other_headers = []
	sources = []
	for p in order:
		ext = os.path.splitext(p)[1].lower()
		if ext in ('.h', '.hpp', '.hh'):
			local_incs, _ = parse_includes(p)
			if not local_incs:
				no_inc_headers.append(p)
			else:
				other_headers.append(p)
		else:
			sources.append(p)
	no_inc_headers.sort()
	new_order = no_inc_headers + other_headers + sources
	order_rel = [os.path.relpath(p, src_dir).replace('\\', '/') for p in new_order]
	gcc_pragmas = []
	seen_pragmas = set()
	for path in new_order:
		for line in collect_gcc_pragmas(path):
			if line not in seen_pragmas:
				seen_pragmas.add(line)
				gcc_pragmas.append(line)
	with open(out_path, 'w', encoding='utf-8', errors='ignore') as out:
		for line in gcc_pragmas:
			out.write(line + '\n')
		if gcc_pragmas:
			out.write('\n')
		out.write(f"// Merged by merge.py on {datetime.utcnow().isoformat()}Z\n")
		out.write(f"// Source dir: {os.path.abspath(src_dir)}\n")
		out.write("// Merged files:\n")
		for r in order_rel:
			out.write(f"//  - {r}\n")
		out.write('\n')

		# system includes
		for inc in sorted(system_includes):
			out.write(f"#include <{inc}>\n")
		out.write('\n')

		written = set()
		for path in new_order:
			if path in written:
				continue
			written.add(path)
			rel = os.path.relpath(path, src_dir).replace('\\', '/')
			out.write(f"// ==== Begin {rel} ====" + '\n')
			try:
				with open(path, 'r', encoding='utf-8', errors='ignore') as f:
					text = f.read()
			except Exception as e:
				if verbose:
					print(f"failed to read {path}: {e}")
				text = ''
			text = strip_local_includes(text)
			text = remove_include_guard(text)
			text = strip_comments(text)
			out.write(text)
			out.write(f"// ==== End {rel} ====" + '\n\n')

	if verbose:
		print(f"merged {len(written)} files -> {out_path}")


def find_compiler():
	preferred = []
	for env_name in ('CXX', 'CMAKE_CXX_COMPILER'):
		value = os.environ.get(env_name)
		if value:
			preferred.append(value)
	preferred.extend([
		'g++',
		'clang++',
		'c++',
		'g++-14',
		'g++-13',
		'g++-12',
		'x86_64-w64-mingw32-g++',
		'clang++-18',
		'clang++-17',
	])
	for candidate in preferred:
		compiler = shutil.which(candidate)
		if compiler:
			return compiler
	return None


def compile_merged(out_path, src_dir, cxx=None, verbose=False):
	# find a suitable compiler (prefer g++ then clang++)
	if cxx is None:
		cxx = find_compiler()
	if not cxx:
		if verbose:
			print('no C++ compiler (g++/clang++) found in PATH; skipping compile')
		return 3

	# prepare output executable path
	out_path = os.path.normpath(out_path)
	src_dir = os.path.normpath(src_dir)
	base = os.path.splitext(os.path.abspath(out_path))[0]
	exe_path = base + ('.exe' if os.name == 'nt' else '')

	cmd = [cxx, '-std=c++17', '-O2', out_path, '-I', src_dir, '-o', exe_path]
	if verbose:
		print('compile command:', ' '.join(cmd))
	proc = subprocess.run(cmd, capture_output=True, text=True)
	if proc.returncode == 0:
		print(f'compile succeeded -> {exe_path}')
	else:
		print('compile failed', file=sys.stderr)
		if proc.stdout:
			print(proc.stdout, file=sys.stderr)
		if proc.stderr:
			print(proc.stderr, file=sys.stderr)
	return proc.returncode


def main():
	parser = argparse.ArgumentParser(description='Merge C/C++ project files into one .cpp')
	parser.add_argument('--src-dir', default='.', help='source directory root')
	parser.add_argument('--main', default='src/main.cpp', help='relative path to main .cpp (relative to src-dir)')
	parser.add_argument('--out', default='src_merge/Solution.cpp', help='output file path')
	parser.add_argument('--exclude', action='append', default=["src/time_log.h"], help='comma-separated filenames (or relative paths) to exclude; can be repeated')
	parser.add_argument('--prepend', action='append', default=["src/off_log.h"], help='comma-separated headers to force merge at the very top; can be repeated')
	parser.add_argument('--verbose', action='store_true')
	parser.add_argument('--no-compile', action='store_true', help='skip compilation step after merge')
	args = parser.parse_args()

	src_dir = os.path.abspath(args.src_dir)
	main_rel = args.main.replace('\\', '/')
	out_path = args.out
	try:
		merge(src_dir, main_rel, out_path, verbose=args.verbose, exclude_list=args.exclude, prepend_list=args.prepend)
	except Exception as e:
		print(f"error: {e}", file=sys.stderr)
		sys.exit(2)

	# attempt to compile merged output unless user opted out
	if not args.no_compile:
		rc = compile_merged(out_path, src_dir, verbose=args.verbose)
		if rc != 0:
			print(f"compile step failed (code {rc})", file=sys.stderr)
			sys.exit(rc)


if __name__ == '__main__':
	main()

