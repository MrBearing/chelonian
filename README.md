# Chelonian

Chelonian is a CLI static analysis tool for ROS workspaces
Usage
-----

Build:

```bash
cargo build --release
```

Run the CLI:

```bash
# analyze a workspace (prints text report)
./target/debug/chel /path/to/ros/workspace

# json output
./target/debug/chel -f json /path/to/ros/workspace

# list available rules
./target/debug/chel --list-rules /path/to/ros/workspace
```

Rules
-----

Rules can be provided as TOML files via `--rules <path>` (file or directory). The tool also loads rules from `~/.config/chelonian/rules/*.toml` and includes several built-in migration rules.

Development
-----------

Run tests:

```bash
cargo test
```

Project layout
--------------

- `src/` — main source files (parsers, scanner, analyzer, plugins, output)
- `Cargo.toml` — dependencies and metadata

Contributing
------------

Please open issues or PRs with improvements, additional rules, or test cases.

License: MIT
