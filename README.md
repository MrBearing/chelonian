# Chelonian

Chelonian is a CLI static analysis tool for ROS workspaces.

Usage (for users)
------------------

Install (from crates.io):

```bash
# install the binary to your cargo bin directory
cargo install chelonian
```

Install (from source):

```bash
# install the binary to your cargo bin directory
cargo install --path . --locked
```

After installation use `chel` from your PATH.

Analyze a workspace (text output):

```bash
chel analyze /path/to/ros/workspace
```

Select a platform (load platform-specific builtin rules):

```bash
# analyze with ROS1 builtin rules
chel analyze --platform ros1 /path/to/ros/workspace

# analyze with ROS2 builtin rules
chel analyze --platform ros2 /path/to/ros/workspace
```

JSON output:

```bash
chel analyze -f json /path/to/ros/workspace
```

Custom rules:

```bash
# add custom rules on top of builtin rules
chel analyze --platform ros1 --rules /path/to/custom/rules /path/to/ros/workspace

# load only custom rules (disable builtin)
chel analyze --rules /path/to/custom/rules --no-builtin /path/to/ros/workspace
```

List available rules:

```bash
chel analyze --list-rules

# include builtin rules for a specific platform
chel analyze -p ros1 --list-rules
chel analyze -p ros2 --list-rules
```

Generate a static HTML report from JSON:

```bash
# 1) analyze and write JSON to a file
chel analyze -p ros1 -f json -o report.json /path/to/ros/workspace

# 2) convert JSON to a standalone HTML file
chel report report.json -o report.html
```

Rules
-----

Rules are provided as TOML files. The tool loads rules in this order:
1. **Builtin rules** (from `builtin-rules/<platform>/`) — selected via `--platform ros1|ros2`
2. **Custom rules** (via `--rules <path>`) — added on top of builtin rules
3. **Config directory** (from `~/.config/chelonian/rules/*.toml`) — added on top

See `RULES.md` for the rule file format and the meaning of the `severity` field (info/warning/error).

Development (for contributors)
------------------------------

Build the project:

```bash
cargo build --release
```

Run tests:

```bash
cargo test
```

Project layout
--------------

- `src/` — main source files (parsers, scanner, analyzer, plugins, output)
- `builtin-rules/` — platform-specific builtin rule sets (ros1/, ros2/)