# libbase64

- Upstream: https://github.com/aklomp/base64
- Release: v0.5.3
- Commit: 5d5074cb856b9a53fe64f04ddf47812a85f4fb5a
- License: BSD-2-Clause (LICENSE)

The lib/ and include/ sources and config.h.in are unmodified upstream files.
CMakeLists.txt is a local integration: a hidden, position-independent static
library embedded in vln_agent_utils. It enables supported x86 SIMD codecs and
ARM64 NEON, keeps the portable fallback, and does not enable OpenMP, a CLI or
upstream installation/export rules. No network access is needed at build time.

The C++ wrapper initializes CPU dispatch once before concurrent use and retains
the previous strict padding/unused-bit validation. Third-party headers and
symbols are not part of the public vln_agent API.
