# OpenFFBoard Host Contract Tests

This durable test crate compiles the board's production
`src/state/stepper.rs` module directly. It does not copy production logic.

The firmware binary cannot use the host test harness because its RTIC and
Cortex-M dependencies contain ARM-only inline assembly, while the thumb target
does not provide Rust's `libtest`. Run the board-pure contracts explicitly on
the development host:

```sh
HOST_TARGET=$(rustc -vV | sed -n 's/^host: //p')
cargo test --manifest-path host-tests/Cargo.toml --target "$HOST_TARGET"
```

The manifest patches transitive git dependencies to the same local component
checkouts used by the aggregate FOCI workspace.
