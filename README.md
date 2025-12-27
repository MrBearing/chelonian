# Chelonian

Chelonian is a CLI static analysis tool for ROS workspaces.

Usage (for users)
------------------

Install (from source):

```bash
# install the binary to your cargo bin directory
cargo install --path . --locked
```

After installation use `chel` from your PATH.

Analyze a workspace (text output):

```bash
chel /path/to/ros/workspace
```

JSON output:

```bash
chel -f json /path/to/ros/workspace
```

List available rules:

```bash
chel --list-rules /path/to/ros/workspace
```

Rules
-----

Rules can be provided as TOML files via `--rules <path>` (file or directory). The tool also loads rules from `~/.config/chelonian/rules/*.toml` and includes several built-in migration rules.

See `RULES.md` for the rule file format and the meaning of the `severity` field (info/warning/error).

Init
----

Create a workspace-local `.chelonian/` directory with example rule templates and output folders:

```bash
# create .chelonian/ in the current workspace and copy bundled rules
chel init

# force overwrite without prompt (non-interactive / CI)
chel init --force

# skip copying example rules
chel init --no-rules
```

After `init`, rule files are placed under `.chelonian/rules/` and analysis results go to `.chelonian/output/` by default. You can also load rules from a custom location with `--rules /path/to/rules` when running the analyzer.

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
- `Cargo.toml` — dependencies and metadata

Contributing
------------

Please open issues or PRs with improvements, additional rules, or test cases.

License: MIT
