# Next.js ESLint glob compatibility

Next.js 16.4.0's ESLint plugin depends on `fast-glob`, which brings the
unpatched `braces` advisory GHSA-vfj7-8cjw-p6xm into the development tree.
The plugin's only use is `globSync(pattern, { onlyDirectories: true })` in
its `get-root-dirs` utility. This local adapter implements that API using
tinyglobby 0.2.17, with directory expansion disabled and directory suffixes
normalized to preserve fast-glob behavior.

The override is scoped to `@next/eslint-plugin-next`. It removes the affected
dependency chain rather than suppressing audit results or disabling rules.
Run `node scripts/verify-lint-dependencies.mjs` to check brace-expanded root
directory globs and confirm that a Next.js lint rule still reports a violation.
Recheck the adapter on any ESLint plugin upgrade; it is not a general-purpose
replacement for all fast-glob APIs.
