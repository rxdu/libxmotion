# Lessons

Recorded per the self-improvement loop: concrete mistakes and the corrections that prevent repeats.

### Intermediate refactor states must honor the target architecture
- **Pattern:** A staged refactor (rename wave → decoupling wave) added `third_party/xmDriver` as a temporary compile bridge in the intermediate wave, directly contradicting the just-decided rule that this component never depends on xmDriver (ADR 0005). The bridge also never compiled (old call sites vs. the new HAL), which CI caught only after the stack was pushed.
- **Correction:** A dependency that the end state forbids may not appear in any intermediate commit either — restructure the waves (merge or reorder) instead of bridging. Build every stack point locally before opening stacked PRs, not just the tip.
- **Context:** C++/CMake, submodule-based composition, stacked-PR refactors in the XMotion family.
