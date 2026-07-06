# Lessons

Recorded per the self-improvement loop: concrete mistakes and the corrections that prevent repeats.

### Intermediate refactor states must honor the target architecture
- **Pattern:** A staged refactor (rename wave → decoupling wave) added `third_party/xmDriver` as a temporary compile bridge in the intermediate wave, directly contradicting the just-decided rule that this component never depends on xmDriver (ADR 0005). The bridge also never compiled (old call sites vs. the new HAL), which CI caught only after the stack was pushed.
- **Correction:** A dependency that the end state forbids may not appear in any intermediate commit either — restructure the waves (merge or reorder) instead of bridging. Build every stack point locally before opening stacked PRs, not just the tip.
- **Context:** C++/CMake, submodule-based composition, stacked-PR refactors in the XMotion family.

### Ignore rules must cover every build-tree naming variant before committing
- **Pattern:** Local verification build trees named `build-iv`/`build-w2` escaped `.gitignore` (which only covered `build`) and were swept into a commit, landing generated CMake caches and binaries in the public history of the merged W2 PR.
- **Correction:** Extend `.gitignore` (e.g. `build-*/`) before creating variant-named build directories, and review `git status` for unexpected paths before any commit that stages broadly.
- **Context:** CMake/C++ repos with multiple out-of-source build configurations.

### Parallel PRs over overlapping paths do not compose semantically
- **Pattern:** PR #47 (delete `controller_interface.hpp`) and PR #48 (rename `common/` → `types/`) were open concurrently; #48 branched before #47 merged, so its `git mv` carried the file to the new path and git's rename-vs-delete resolution silently resurrected it after both merges.
- **Correction:** PRs touching overlapping paths must be stacked (later branched on the earlier) or serialized (second opened only after the first merges). After merging concurrent PRs, verify the composed tree matches both intents — git only guarantees textual, not semantic, composition.
- **Context:** git merge semantics; multi-PR workflows in the XMotion family.
