// Next's ESLint plugin only imports globSync for its rootDir setting.
// tinyglobby expands matched directory contents by default; fast-glob does not.
const { globSync } = require('tinyglobby');
exports.globSync = (patterns, options = {}) =>
  globSync(patterns, { ...options, expandDirectories: false })
    .map(p => p.length > 1 && !options.markDirectories ? p.replace(/[\\/]$/, '') : p);
