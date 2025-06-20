# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]

- Update `e310x-hal` dependency and adapt code

## [v0.13.0] - 2024-12-10

- Fix Led implementation, as pins are configured as inverted outputs
- Adapt to embedded-hal 1.0
- Replace static muts with Mutexes
- Apply clippy changes
- Bump MSRV to 1.76
- Adapt to new Cargo workspace
- Use inline assembly instead of binary blobs for flash

## [v0.12.0] - 2023-03-28
- Update e310x-hal to v0.11 with new svd2rust generated code

## [v0.11.0] - 2023-03-03

### Changed
- Updated riscv dependency to v0.10 with interrupt/critical section changes

## [v0.10.0] - 2021-07-15

### Added

- Added [SparkFun Red-V RedBoard](https://www.sparkfun.com/products/15594)` support
