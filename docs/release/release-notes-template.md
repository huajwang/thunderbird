# Release Notes — v{VERSION}

**Release date:** YYYY-MM-DD

---

## Highlights

<!--
  One-paragraph summary of the release for quick scanning.
  Focus on the user-visible impact, not implementation details.
-->

---

## New Features

- **Feature Name** — one-line description.
  ([#PR](https://github.com/huajwang/thunderbird/pull/NN))

---

## Improvements

- Improved X by Y%.
  ([#PR](https://github.com/huajwang/thunderbird/pull/NN))

---

## Bug Fixes

- Fixed crash when Z.
  ([#PR](https://github.com/huajwang/thunderbird/pull/NN))

---

## Breaking Changes

<!--
  List any API, ABI, or behavioural changes that require user action.
  If none, write "None." and remove the migration section.
-->

- `OldFunction()` renamed to `NewFunction()`.

### Migration Guide

```cpp
// Before (v{OLD_VERSION})
auto result = thunderbird::OldFunction();

// After (v{VERSION})
auto result = thunderbird::NewFunction();
```

---

## Deprecations

- `DeprecatedAPI()` — will be removed in v{NEXT_MAJOR}. Use `NewAPI()`.

---

## Known Issues

- Issue description. ([#ISSUE](https://github.com/huajwang/thunderbird/issues/NN))

---

## Dependencies

| Dependency | Previous | New |
|-----------|----------|-----|
| spdlog | 1.10.0 | 1.12.0 |

---

## Compatibility

| Platform | Compiler | Status |
|----------|----------|--------|
| Ubuntu 22.04 | GCC 12 | ✅ Tested |
| Ubuntu 24.04 | GCC 14 | ✅ Tested |
| Debian 12 | GCC 12 | ✅ Tested |
| macOS 14 | Apple Clang 15 | ✅ Tested |
| Windows (Cygwin) | GCC 11 | ✅ Tested |

---

## Contributors

- @contributor — description of contribution

---

## Full Changelog

[v{PREV_VERSION}...v{VERSION}](https://github.com/huajwang/thunderbird/compare/v{PREV_VERSION}...v{VERSION})
